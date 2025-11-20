# Copyright: (C) 2025 iCub Facility - Istituto Italiano di Tecnologia
# Authors: SATHISH KUMAR S
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# yafu - Yet_Another_Firmware_Updater
# version: 1.0 - 2025/11/17

#!/bin/bash

# This script discovery for multiple boards in parallel.
# Usage: ./program_boards.sh <ip1> <ip2> ... <ipN>

# directory of this script
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ensure logs directory exists and run from it
mkdir -p "$script_dir/logs"
cd "$script_dir/logs"

# Check if at least one IP address is provided.
if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <ip1> <ip2> ... <ipN>"
    exit 1
fi

# Resolve yafu executable:
if [ -n "${YAFU_EXECUTABLE:-}" ]; then
    EXECUTABLE="$YAFU_EXECUTABLE"
else
    EXECUTABLE="$(command -v yafu 2>/dev/null || true)"
fi

# Fallback: search upward for a local build copy (development convenience)
if [ -z "$EXECUTABLE" ]; then
    search_dir="$script_dir"
    while [ "$search_dir" != "/" ]; do
        if [ -x "$search_dir/build/src/ICUB/bin/yafu" ]; then
            EXECUTABLE="$search_dir/build/src/ICUB/bin/yafu"
            break
        fi
        if [ -x "$search_dir/build/bin/yafu" ]; then
            EXECUTABLE="$search_dir/build/bin/yafu"
            break
        fi
        if [ -x "$search_dir/bin/yafu" ]; then
            EXECUTABLE="$search_dir/bin/yafu"
            break
        fi
        search_dir="$(dirname "$search_dir")"
    done
fi

# Verify executable found
if [ -z "$EXECUTABLE" ] || [ ! -x "$EXECUTABLE" ]; then
    echo "yafu executable not found."
    echo "Either install yafu so it's on PATH, set YAFU_EXECUTABLE to its path, or build yafu in a nearby build directory."
    exit 1
fi

# Loop through all IP addresses provided as arguments.
for ip in "$@"; do
    echo "Starting discovering for board at $ip..."
    "$EXECUTABLE" "$ip" discover > "discover_${ip}.log" 2>&1 &
    # Add a small delay to stagger the process launches.
    # sleep 0.1
done

# The 'wait' command pauses the script until all background jobs launched by this script have finished.
echo "Waiting for all discovering processes to complete..."
wait

echo "All discovering tasks are complete. Check the discover_<ip>.log files for details."
# ...existing code...