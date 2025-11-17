# Copyright: (C) 2025 iCub Facility - Istituto Italiano di Tecnologia
# Authors: SATHISH KUMAR S
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# yafu - Yet_Another_Firmware_Updater
# version: 1.0 - 2025/11/17

#!/bin/bash

# This script programs multiple boards in parallel.
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

# Resolve yafu executable: env override, then PATH, then local-build fallback
if [ -n "${YAFU_EXECUTABLE:-}" ]; then
    EXECUTABLE="$YAFU_EXECUTABLE"
else
    EXECUTABLE="$(command -v yafu 2>/dev/null || true)"
fi

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

# Check if the executable exists and is executable.
if [ -z "$EXECUTABLE" ] || [ ! -x "$EXECUTABLE" ]; then
    echo "Error: yafu executable not found or is not executable."
    echo "Please ensure it is installed and on PATH, set YAFU_EXECUTABLE, or build yafu in a nearby build directory."
    exit 1
fi

# Loop through all IP addresses provided as arguments.
for ip in "$@"; do
    echo "Starting programming for board at $ip..."
    "$EXECUTABLE" "$ip" program > "program_${ip}.log" 2>&1 &
    # Add a small delay to stagger the process launches.
    # sleep 0.1
done

# The 'wait' command pauses the script until all background jobs launched by this script have finished.
echo "Waiting for all programming processes to complete..."
wait

echo "All programming tasks are complete. Check the program_<ip>.log files for details."