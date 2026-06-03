"""SiteScope HTTP client for the Go2W robot stack.

A lightweight wrapper around SiteScope's REST API, called by the mission
scheduler and capture scripts on the Jetson Orin NX. It has no dependency on
ROS2 or the robot stack, only ``requests``, so it can be unit-tested and run
from a laptop against a running SiteScope instance.

Phase 1 scope (see SiteScope Robot Integration Plan):
  * authenticate()           , log in, hold a JWT, refresh before expiry
  * get_waypoints(phase)     , rooms to visit; local JSON until IFC is wired
  * upload_file(path, meta)  , push a capture with pose / mission metadata

Small captures (panorama JPEGs) post in one request to ``POST /api/upload/robot``.
Large point clouds (LAZ above the 32 MB threshold) use SiteScope's existing
chunked protocol (``/pointcloud/init`` → ``/pointcloud/chunk`` →
``/pointcloud/complete``): each 32 MB slice is an independent request, so a
mid-transfer wifi drop only re-sends the failed chunk, not the whole file. Both
paths carry the same ``robot_meta`` and produce a ``source: robot`` activity
entry. Failed requests are retried with backoff.

Usage:
    client = SiteScopeClient("http://192.168.50.x:3004", "go2w_001", "secret")
    client.authenticate()
    client.upload_file(
        "/tmp/pano.jpg",
        project_slug="a6-stern",
        room_slug="ground-floor",
        media_type="image",
        capture_date="2026-06-03",
        robot_meta={"pose": {...}, "mission_id": "m-42", "sensor": "insta360"},
    )
"""

from __future__ import annotations

import json
import logging
import os
import time
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone

import requests

logger = logging.getLogger("sitescope_client")

# Re-authenticate when less than this remains on the token (plan §2.2.4).
_TOKEN_REFRESH_MARGIN = timedelta(hours=1)
# JWTs are HS256 / 7-day lifetime (see ARCHITECTURE.md). We don't decode the
# token; we just track when we obtained it and assume the documented lifetime.
_TOKEN_LIFETIME = timedelta(days=7)

# Point clouds larger than this use the chunked upload protocol; smaller files
# (and all other media types) go in a single request. Matches the backend's
# 32 MB pointcloud chunk size.
_CHUNK_THRESHOLD = 32 * 1024 * 1024
_CHUNK_SIZE = 32 * 1024 * 1024


class SiteScopeError(RuntimeError):
    """Raised when SiteScope returns an unexpected response."""


@dataclass
class SiteScopeClient:
    base_url: str
    username: str
    password: str
    # Local waypoint config used until the IFC waypoint API is available
    # (plan §3.4). Same output shape as GET .../ifc/waypoints.
    waypoints_path: str | None = None
    timeout: float = 30.0
    upload_timeout: float = 600.0  # large LAZ files over wifi
    max_retries: int = 4

    _token: str | None = field(default=None, init=False, repr=False)
    _token_acquired_at: datetime | None = field(default=None, init=False, repr=False)

    # -- auth ---------------------------------------------------------------

    def authenticate(self) -> None:
        """Log in and cache the JWT. Safe to call repeatedly."""
        resp = requests.post(
            f"{self.base_url}/api/auth/login",
            json={"username": self.username, "password": self.password},
            timeout=self.timeout,
        )
        if resp.status_code != 200:
            raise SiteScopeError(
                f"Login failed ({resp.status_code}): {resp.text[:200]}"
            )
        self._token = resp.json()["access_token"]
        self._token_acquired_at = datetime.now(timezone.utc)
        logger.info("Authenticated to SiteScope as %s", self.username)

    def _ensure_token(self) -> str:
        """Return a valid token, re-authenticating if missing or near expiry."""
        if self._token is None or self._token_acquired_at is None:
            self.authenticate()
        else:
            age = datetime.now(timezone.utc) - self._token_acquired_at
            if age > (_TOKEN_LIFETIME - _TOKEN_REFRESH_MARGIN):
                logger.info("Token nearing expiry, re-authenticating")
                self.authenticate()
        assert self._token is not None
        return self._token

    def _auth_header(self) -> dict[str, str]:
        return {"Authorization": f"Bearer {self._ensure_token()}"}

    # -- waypoints ----------------------------------------------------------

    def get_waypoints(self, phase: int | str, project_slug: str | None = None) -> list[dict]:
        """Return the rooms the robot should visit for this construction phase.

        Until the IFC waypoint API exists, this reads a local JSON config on the
        Jetson (``waypoints_path``). The mission scheduler does not need to know
        which source was used, the output shape is identical:

            [{"room_slug": "ground-floor", "name": "Ground Floor",
              "x": 1.2, "y": 3.4, "phase": 1}, ...]
        """
        if self.waypoints_path:
            return self._load_local_waypoints(phase)
        # IFC-driven path (plan §5), enabled once the endpoint exists.
        if project_slug is None:
            raise SiteScopeError("project_slug required for the IFC waypoint API")
        resp = requests.get(
            f"{self.base_url}/api/projects/{project_slug}/ifc/waypoints",
            params={"phase": phase},
            headers=self._auth_header(),
            timeout=self.timeout,
        )
        if resp.status_code != 200:
            raise SiteScopeError(
                f"Waypoint fetch failed ({resp.status_code}): {resp.text[:200]}"
            )
        return resp.json()

    def _load_local_waypoints(self, phase: int | str) -> list[dict]:
        with open(self.waypoints_path, encoding="utf-8") as fh:  # type: ignore[arg-type]
            data = json.load(fh)
        rooms = data if isinstance(data, list) else data.get("waypoints", [])
        # Filter by phase when the config declares one; otherwise return all.
        filtered = [
            r for r in rooms
            if "phase" not in r or str(r["phase"]) == str(phase)
        ]
        logger.info("Loaded %d waypoint(s) for phase %s from local config", len(filtered), phase)
        return filtered

    # -- upload -------------------------------------------------------------

    def upload_file(
        self,
        file_path: str,
        *,
        project_slug: str,
        room_slug: str,
        media_type: str,
        capture_date: str | None = None,
        robot_meta: dict | None = None,
    ) -> dict:
        """Upload one capture to SiteScope with mission metadata.

        ``media_type`` is one of image / video / pointcloud / pdf.
        ``capture_date`` is an ISO date (YYYY-MM-DD); defaults to today (UTC).
        Returns the parsed UploadResponse JSON.
        """
        if not os.path.isfile(file_path):
            raise FileNotFoundError(file_path)
        capture_date = capture_date or datetime.now(timezone.utc).date().isoformat()

        # Large point clouds use the resilient chunked protocol; everything else
        # (panoramas, small clouds) goes in one request.
        if media_type == "pointcloud" and os.path.getsize(file_path) > _CHUNK_THRESHOLD:
            return self._chunked_pointcloud_upload(
                file_path,
                project_slug=project_slug,
                room_slug=room_slug,
                capture_date=capture_date,
                robot_meta=robot_meta,
            )

        data = {
            "project_slug": project_slug,
            "room_slug": room_slug,
            "media_type": media_type,
            "capture_date": capture_date,
            "robot_meta": json.dumps(robot_meta or {}),
        }

        last_err: Exception | None = None
        for attempt in range(1, self.max_retries + 1):
            try:
                with open(file_path, "rb") as fh:
                    resp = requests.post(
                        f"{self.base_url}/api/upload/robot",
                        data=data,
                        files={"file": (os.path.basename(file_path), fh)},
                        headers=self._auth_header(),
                        timeout=self.upload_timeout,
                    )
                if resp.status_code == 409:
                    # Duplicate, the same file already exists. Not an error worth
                    # retrying; treat as success so the mission continues.
                    logger.warning("Duplicate upload for %s: %s", file_path, resp.text[:200])
                    return {"duplicate": True, "detail": resp.json().get("detail")}
                if resp.status_code in (200, 201):
                    logger.info("Uploaded %s -> %s/%s", file_path, project_slug, room_slug)
                    return resp.json()
                if resp.status_code == 401:
                    # Token expired mid-mission; force refresh and retry.
                    logger.info("401 on upload, refreshing token")
                    self.authenticate()
                    raise SiteScopeError("re-auth, retrying")
                raise SiteScopeError(f"Upload failed ({resp.status_code}): {resp.text[:200]}")
            except (requests.RequestException, SiteScopeError) as exc:
                last_err = exc
                backoff = min(2 ** attempt, 30)
                logger.warning(
                    "Upload attempt %d/%d failed (%s); retrying in %ds",
                    attempt, self.max_retries, exc, backoff,
                )
                time.sleep(backoff)

        raise SiteScopeError(f"Upload of {file_path} failed after {self.max_retries} attempts: {last_err}")

    # -- chunked upload (large point clouds) --------------------------------

    def _resolve_room_id(self, room_slug: str) -> str:
        """Look up a room's id from its slug (chunked init needs the id)."""
        resp = requests.get(
            f"{self.base_url}/api/rooms/{room_slug}",
            headers=self._auth_header(),
            timeout=self.timeout,
        )
        if resp.status_code != 200:
            raise SiteScopeError(
                f"Room lookup for '{room_slug}' failed ({resp.status_code}): {resp.text[:200]}"
            )
        return resp.json()["id"]

    def _post_with_retry(self, url: str, *, what: str, **kwargs) -> requests.Response:
        """POST with the same backoff/re-auth policy as upload_file."""
        last_err: Exception | None = None
        for attempt in range(1, self.max_retries + 1):
            try:
                headers = {**kwargs.pop("headers", {}), **self._auth_header()}
                resp = requests.post(url, headers=headers, timeout=self.upload_timeout, **kwargs)
                if resp.status_code in (200, 201):
                    return resp
                if resp.status_code == 401:
                    self.authenticate()
                    raise SiteScopeError("re-auth, retrying")
                raise SiteScopeError(f"{what} failed ({resp.status_code}): {resp.text[:200]}")
            except (requests.RequestException, SiteScopeError) as exc:
                last_err = exc
                backoff = min(2 ** attempt, 30)
                logger.warning("%s attempt %d/%d failed (%s); retry in %ds",
                               what, attempt, self.max_retries, exc, backoff)
                time.sleep(backoff)
        raise SiteScopeError(f"{what} failed after {self.max_retries} attempts: {last_err}")

    def _chunked_pointcloud_upload(
        self,
        file_path: str,
        *,
        project_slug: str,
        room_slug: str,
        capture_date: str,
        robot_meta: dict | None,
    ) -> dict:
        """Upload a large LAZ via /pointcloud/{init,chunk,complete}.

        Each chunk is an independent request, so a transient wifi drop only
        costs a single failed chunk (auto-retried) rather than the whole file.
        ``project_slug`` is accepted for a symmetric signature; the chunked
        endpoints key off room_id, which we resolve from the slug (room slugs
        are globally unique in SiteScope).
        """
        room_id = self._resolve_room_id(room_slug)
        file_size = os.path.getsize(file_path)
        filename = os.path.basename(file_path)

        init = self._post_with_retry(
            f"{self.base_url}/api/upload/pointcloud/init",
            what="Chunked init",
            data={
                "room_id": room_id,
                "capture_date": capture_date,
                "filename": filename,
                "file_size": file_size,
                "content_type": "application/octet-stream",
                "robot_meta": json.dumps(robot_meta or {}),
            },
        ).json()
        upload_id = init["upload_id"]
        chunk_size = int(init.get("chunk_size", _CHUNK_SIZE))

        chunk_index = 0
        with open(file_path, "rb") as fh:
            while True:
                blob = fh.read(chunk_size)
                if not blob:
                    break
                self._post_with_retry(
                    f"{self.base_url}/api/upload/pointcloud/chunk",
                    what=f"Chunk {chunk_index}",
                    data={"upload_id": upload_id, "chunk_index": chunk_index},
                    files={"chunk": (f"{chunk_index:08d}.part", blob)},
                )
                chunk_index += 1

        resp = self._post_with_retry(
            f"{self.base_url}/api/upload/pointcloud/complete",
            what="Chunked complete",
            data={"upload_id": upload_id, "total_chunks": chunk_index},
        )
        logger.info("Chunked-uploaded %s (%d chunks) -> %s/%s",
                    file_path, chunk_index, project_slug, room_slug)
        return resp.json()
