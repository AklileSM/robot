# Robot integration (Go2W → SiteScope)

Robot-side code that bridges the Unitree Go2W stack (ROS2 + Nav2 + LLM) to
SiteScope. This is the **Phase 1** bridge from the *SiteScope Robot Integration
Plan*: the robot navigates, captures, and uploads; SiteScope stores, analyses,
and presents.

This folder is meant to be copied to / vendored into the robot codebase on the
Jetson Orin NX. It has no ROS2 dependency, only `requests`, so it can be tested
from a laptop against a running SiteScope instance.

## Contents

| File | Role |
| --- | --- |
| `sitescope_client.py` | HTTP client: `authenticate()`, `get_waypoints(phase)`, `upload_file(...)` |
| `waypoints.example.json` | Manual waypoint config used until the IFC waypoint API exists (plan §3.4) |

## Backend prerequisites (already implemented in `backend/`)

1. `users.is_robot` column + `ensure_users_is_robot` migration (runs at startup).
2. Admin endpoint to mint a robot service account:
   `POST /api/admin/robot-accounts` `{ "username": "go2w_001", "password": "..." }`
   — creates a pre-verified, non-admin, `is_robot=true` user.
3. Robot upload endpoint: `POST /api/upload/robot` (requires `is_robot=true` JWT),
   accepts `project_slug`, `room_slug`, `media_type`, `capture_date`, `file`,
   and a `robot_meta` JSON blob. Stores `robot_meta` under
   `file_assets.metadata_json.robot`, tags the activity feed with
   `source: robot`, and triggers the existing AI analysis + PotreeConverter
   pipelines unchanged.

## One-time setup

```bash
# 1. As a SiteScope admin, create the robot account (via the API or admin panel):
curl -X POST https://sitescope.example/api/admin/robot-accounts \
  -H "Authorization: Bearer <ADMIN_JWT>" \
  -H "Content-Type: application/json" \
  -d '{"username": "go2w_001", "password": "<strong-password>"}'

# 2. Add go2w_001 as an owner/editor member of the target project
#    (admin panel → project members), so it is allowed to upload.
```

## End-to-end smoke test (do this before any robot code)

```python
from sitescope_client import SiteScopeClient

client = SiteScopeClient(
    base_url="http://<sitescope-host>:3004",
    username="go2w_001",
    password="<strong-password>",
    waypoints_path="waypoints.json",   # local config until IFC is wired
)
client.authenticate()

# Upload a test image from your laptop and confirm it appears in SiteScope,
# in the right room, with robot metadata and a "source: robot" activity entry.
client.upload_file(
    "test.jpg",
    project_slug="a6-stern",
    room_slug="ground-floor",
    media_type="image",
    capture_date="2026-06-03",
    robot_meta={
        "pose": {"x": 1.2, "y": 3.4, "yaw": 0.0, "frame": "map"},
        "mission_id": "smoke-test",
        "sensor": "manual",
    },
)
```

If that image shows up in the platform, authentication, the endpoint, and
metadata storage are all working — the bridge is proven. From there, Phase 1b
(panorama capture) and Phase 1c (LiDAR) add `panorama_capture()`,
`lidar_capture()`, and `mission_scheduler.py` on top of this client.

## Notes

- **Large files**: `upload_file()` automatically switches to SiteScope's chunked
  protocol (`/pointcloud/{init,chunk,complete}`) for point clouds above 32 MB.
  Each 32 MB slice is an independent request, so a mid-transfer wifi drop only
  re-sends the failed chunk (auto-retried), not the whole 80–200 MB file. Smaller
  files and all other media types go in a single `POST /api/upload/robot`. Both
  paths carry `robot_meta` and tag the activity feed `source: robot`.
- **Waypoints**: `get_waypoints()` reads the local JSON until the IFC waypoint
  API (`GET /api/projects/{slug}/ifc/waypoints?phase=`) lands; the output shape
  is identical so the mission scheduler is source-agnostic.
