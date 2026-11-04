#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ${EUID} -ne 0 ]]; then
    exec sudo /usr/bin/python3 "${script_dir}/configure.py" "$@"
fi
exec /usr/bin/python3 "${script_dir}/configure.py" "$@"
