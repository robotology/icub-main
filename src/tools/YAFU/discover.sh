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

# Path to your executable .
EXECUTABLE="$script_dir/../../../../../build/src/ICUB/bin/YAFU"

# Check if the executable exists and is executable.
if [ ! -x "$EXECUTABLE" ]; then
    echo "Error: '$EXECUTABLE' not found or is not executable."
    echo "Please ensure it is built and in the current directory."
    exit 1
fi

# Loop through all IP addresses provided as arguments.
for ip in "$@"; do
    echo "Starting discovering for board at $ip..."
    # Run the discover command in the background (&) for each IP.
    # Redirect output to a log file for each board.
    $EXECUTABLE "$ip" discover > "discover_${ip}.log" 2>&1 &
    # Add a small delay to stagger the process launches.
    # sleep 0.1
done

# The 'wait' command pauses the script until all background jobs launched by this script have finished.
echo "Waiting for all discovering processes to complete..."
wait

echo "All discovering tasks are complete. Check the discover_<ip>.log files for details."