#!/bin/bash
set -euo pipefail

# directory of this script
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Resolve yafu executable: env override, then PATH, then local-build fallback
if [ -n "${YAFU_EXECUTABLE:-}" ]; then
    candidate="$YAFU_EXECUTABLE"
else
    candidate="$(command -v yafu 2>/dev/null || true)"
fi

if [ -z "$candidate" ]; then
    search_dir="$script_dir"
    while [ "$search_dir" != "/" ]; do
        if [ -x "$search_dir/build/src/ICUB/bin/yafu" ]; then
            candidate="$search_dir/build/src/ICUB/bin/yafu"
            break
        fi
        if [ -x "$search_dir/build/bin/yafu" ]; then
            candidate="$search_dir/build/bin/yafu"
            break
        fi
        if [ -x "$search_dir/bin/yafu" ]; then
            candidate="$search_dir/bin/yafu"
            break
        fi
        search_dir="$(dirname "$search_dir")"
    done
fi

if [ -z "$candidate" ] || [ ! -x "$candidate" ]; then
  echo "Error: yafu not found or not executable."
  echo "Expected yafu on PATH, set YAFU_EXECUTABLE to its path, or build yafu in a nearby build dir."
  exit 1
fi

# run from the script directory so logs created by the program end up here
mkdir -p "$script_dir/logs"
cd "$script_dir/logs"

NETWORK_XML="$(yarp resource --from network.$YARP_ROBOT_NAME.xml | grep '^\".*$' | sed 's/\"//g')"
# NETWORK_XML="$script_dir/../network.$YARP_ROBOT_NAME.xml"  # alternative: fixed path

echo "Running: $candidate parallel_program (network file: $NETWORK_XML, logs -> $(pwd))"
# pass NETWORK_XML via environment
NETWORK_XML="$NETWORK_XML" "$candidate" parallel_program