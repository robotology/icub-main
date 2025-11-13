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

EXECUTABLE="$script_dir/../../../../../build/src/ICUB/bin/YAFU"

for ip in "$@"; do
    echo "Starting restart for board at $ip..."
    $EXECUTABLE "$ip" restart > "restart_${ip}.log" 2>&1 &
    # optional small delay to stagger launches:
    # sleep 0.1
done

echo "Waiting for all restart processes to complete..."
wait

echo "All restart tasks are complete. Check the restart_<ip>.log files for details."