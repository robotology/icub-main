#!/bin/bash
set -euo pipefail


# directory of this script
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# expected built binary (relative from this folder)
candidate="$script_dir/../../../../../build/src/ICUB/bin/YAFU"


if [ -z "$candidate" ] || [ ! -x "$candidate" ]; then
  echo "Error: YAFU not found or not executable."
  echo "Expected at: $script_dir/../../../../../build/src/ICUB/bin/YAFU"
  exit 1
fi

# run from the script directory so logs created by the program end up here
mkdir -p "$script_dir/logs"
cd "$script_dir/logs"

NETWORK_XML="$(yarp resource --from network.$YARP_ROBOT_NAME.xml | grep '^\".*$' | sed 's/\"//g')"
# NETWORK_XML="/home/sk/development/robotology-superbuild/src/icub-firmware-build/scripts/network.setupFU.xml"

echo "Running: $candidate parallel_program (network file: $NETWORK_XML, logs -> $(pwd))"
# pass NETWORK_XML via environment
NETWORK_XML="$NETWORK_XML" "$candidate" parallel_program
