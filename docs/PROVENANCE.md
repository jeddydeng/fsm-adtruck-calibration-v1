# Source and change provenance

Packaged from the user's Mac backups:

- white_apply/white_ws/src: deployed vehicle sensor drivers and sensor-kit TF/config,
  snapshot September 4, 2026.
- white_apply/white_ws/calibration: camera capture/ordinary intrinsic entry scripts.
- calibration-white/process: offline extrinsic, robust fisheye and BEV solvers,
  September 2–3, 2026.
- calibration-white/result: parameter snapshots, reports and BEV numeric maps.
- calibration-white/calibration/runtime_context_20260902/Pi-capture:
  capture-station-compatible network SDK and protocol.
- FSM-Pi-master and FSM-Pi-001: September 4 Pi receiver/sender source releases.

The vehicle computer was powered off during initial packaging. A read-only SSH
comparison was completed on November 5, 2026 after it returned online; see
VEHICLE_SOURCE_AUDIT_20261105.md. The September 17 vehicle IMU correction configuration
was adopted with the owner's explicit approval. Original backup directories and all
vehicle files were left unchanged. This is a portable calibration repository, not a
byte-identical backup of the entire live white_ws or a deployment to the vehicle.

Packaging changes: repository-relative paths; independent minimal ROS bringup;
unified entry points; robust equidistant path for all four fisheyes; pinned Python
dependencies; portable installer; process ownership/cleanup; loopback web binding;
correct package-scoped public serial-header installation for Humble/Jazzy;
new software/transport/coordinate checks and documentation.
OpenCV AprilTag construction compatibility and BEV deployment-status consistency
were also fixed and covered by regression tests. See VALIDATION.md for observed results.

result/ retains historical candidate status. Non-deployable results have not been
relabeled. Raw photos, PCD, videos, logs, caches, virtual environments and compilation
outputs are excluded. Numeric remap arrays and configuration metadata are retained.
Some historical report links reference excluded imagery; reproduce those previews
from the original local dataset. New outputs belong in output/.
