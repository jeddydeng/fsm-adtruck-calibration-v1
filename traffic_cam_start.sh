#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${root}/start_joint.sh" --traffic-only "$@"
