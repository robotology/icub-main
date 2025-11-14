#!/bin/bash

# This script restarts multiple boards in parallel.
# Usage: ./restart.sh <ip1> <ip2> ... <ipN>

# directory of this script
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ensure logs directory exists and run from it
mkdir -p "$script_dir/logs"
cd "$script_dir/logs"

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

if [ -z "$EXECUTABLE" ] || [ ! -x "$EXECUTABLE" ]; then
    echo "yafu executable not found."
    echo "Either install yafu so it's on PATH, set YAFU_EXECUTABLE to its path, or build yafu in a nearby build directory."
    exit 1
fi

for ip in "$@"; do
    echo "Starting restart for board at $ip..."
    "$EXECUTABLE" "$ip" restart > "restart_${ip}.log" 2>&1 &
    # optional small delay to stagger launches:
    # sleep 0.1
done

echo "Waiting for all restart processes to complete..."
wait

echo "All restart tasks are complete. Check the restart_<ip>.log files for details."