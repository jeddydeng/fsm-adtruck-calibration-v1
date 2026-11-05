# Repository branch policy

- The active development branch for this vehicle is `white_truck`.
- Put subsequent code, configuration, calibration and documentation changes on
  `white_truck` unless the user explicitly requests a different branch.
- Before editing or committing, inspect `git status --short --branch` and confirm
  the active branch. Preserve any existing user changes; do not switch branches
  over conflicting changes or reset them.
- `main` is the original published baseline (file tree of commit `0b253a5`). Do not
  merge vehicle changes into `main` or rewrite its history without an explicit request.
- Run the relevant checks before publishing changes. When tracked source/configuration
  files change, regenerate `SOURCE_MANIFEST.sha256` using
  `scripts/create_source_manifest.py` after staging new files, then stage the manifest.
- Branch operations do not authorize deployment to the vehicle, starting hardware,
  arming services or CAN transmission.
