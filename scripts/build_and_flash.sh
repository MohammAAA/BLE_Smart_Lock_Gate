#!/bin/bash
# This bash script automates the following steps:
#   0- sources and activates zephyr environment (for west usage)
#   1- clean the build dir
#   2- build the software for promicro_nrf52840 using --pristine to clear cached data
#   3- convert the output hex file to uf2 file
#   4- flash the firmware
#   5- If user wants to trigger monitor, start cat command. Can optionally save and clean logs.

# Example 1: ./scripts/build_and_flash.sh --source-zephyr # Just build and flash
# Example 2: ./scripts/build_and_flash.sh --monitor # Monitors using cat without saving logs
# Example 3: ./scripts/build_and_flash.sh --save-log # Monitors using cat and saving the log file to the default path and with default name
# Example 4: ./scripts/build_and_flash.sh --save-log --custom-path ./my_logs/custom_reading.log # Monitors using cat and saving the log file to a custom path

# Exit immediately if a command exits with a non-zero status.
# This ensures that if a step like 'west build' fails, it won't proceed to flash.
set -e

# Variables to track monitor and logging
START_MONITOR=0
SAVE_LOG=0
CUSTOM_PATH=""
# whether to source zephyr environment or not.
SOURCE_ZEPHYR=0

# Parse arguments
while [[ $# -gt 0 ]]
do
    case $1 in
        --monitor)
        START_MONITOR=1
        shift # This shifts all command-line arguments one position to the left (e.g., what was argument 2 becomes argument 1). It's commonly used to "consume" arguments once they've been processed.
        ;; # equavilent to 'break' in C
        --save-log)
        START_MONITOR=1
        SAVE_LOG=1
        shift
        ;;
        --custom-path)
        # If the next argument is not another flag, then it is a valid path
        if [ -n "$2" ] && [[ "$2" != -* ]]; then
            CUSTOM_PATH="$2"
            shift 2
        else
            # The next argument doesn't include a valid path, exit with error
            echo "LOG Error: --custom-path requires a valid file path argument."
            exit 1
        fi
        ;;
        -s|--source-zephyr)
        SOURCE_ZEPHYR=1
        shift
        ;;
        *)
        shift
        ;;
    esac
done

# Change to the project root directory so that all relative paths work
# regardless of where this script is called from.
cd "$(dirname "$0")/.."
# Explanation of the previous statement:
# "$0" is a special variable that holds the path used to call the script (e.g.: ./scripts/build_and_flash.sh)
# 'dirname' is a standard Linux command that strips the filename off a path, leaving only the directory.
# So if $0 is ./scripts/build_and_flash.sh, dirname "$0" returns ./scripts.
# $( ... ): This is called "command substitution". It runs the command inside the parentheses
# and replaces itself with the output. So $(dirname "$0") gets replaced by ./scripts.
# /..: In Linux, .. always refers to the parent directory (one folder up).

# Putting it together:
#  It translates to: "Find out exactly which folder this script is located in
#  (./scripts), navigate one folder up from there (..), and change the current
#  working directory (cd) to that location."

#  This is how we can run ./scripts/build_and_flash.sh from the root folder,
#  or (cd scripts && ./build_and_flash.sh), and west
#  build will still perfectly find our project!
# --------------------------------------------------------------------------------------------------


if [ $SOURCE_ZEPHYR -eq 1 ]; then
    if [ -z "$ZEPHYR_ENV_PATH" ]; then
        echo "LOG Error: --source-zephyr flag was used, but the ZEPHYR_ENV_PATH environment variable is not set."
        echo "Please set it before running the script, for example:"
        echo "export ZEPHYR_ENV_PATH=/path/to/zephyrproject/.zephyrenv/bin/activate"
        exit 1
    fi
    echo "LOG: Sourcing Zephyr environment from $ZEPHYR_ENV_PATH..."
    source "$ZEPHYR_ENV_PATH"
fi

echo "LOG: 1. Cleaning build directory... "
rm -rf build

echo "LOG: 2. Building for promicro_nrf52840..."
west build --pristine -b promicro_nrf52840/nrf52840 .

echo "LOG: 3. Converting firmware to uf2 format..."
python3 scripts/uf2conv.py build/zephyr/zephyr.hex -c -f 0xADA52840 -o build/zephyr/zephyr.uf2

echo "LOG: 4. Flashing firmware..."
./scripts/flash.sh

echo "LOG: Flashing successful!"

if [ $START_MONITOR -eq 1 ]; then
    echo "Waiting for 5 seconds before starting monitor .. "
    sleep 5
    echo "LOG: 5. Starting monitor on /dev/ttyACM0..."
    stty -F /dev/ttyACM0 raw -echo
    if [ $SAVE_LOG -eq 1 ]; then
        if [ -z "$CUSTOM_PATH" ]; then
            mkdir -p ./logs
            TIMESTAMP=$(date +"%Y%m%d%H%M%S")
            LOG_FILE="./logs/${TIMESTAMP}.log"
        else
            LOG_DIR=$(dirname "$CUSTOM_PATH")
            mkdir -p "$LOG_DIR"
            LOG_FILE="$CUSTOM_PATH"
        fi
        echo "LOG: Saving log to $LOG_FILE"
        
        # This defines a custom Bash function named "cleanup".
        # In Bash, we can group multiple commands together inside a function block.
        # We put the sed log-cleaning commands and the
        # final print statements inside this function so that they can be easily
        # triggered as a single action later.
        cleanup() {
            echo ""
            echo "LOG: Cleaning log file..."
            CLEAN_LOG_FILE="${LOG_FILE%.log}_clean.log"
            sed 's/\r$//' "$LOG_FILE" | sed 's/\x1b\[[0-9;?]*[a-zA-Z]//g' > "$CLEAN_LOG_FILE"
            echo "LOG: Clean log saved to $CLEAN_LOG_FILE"
            # Tell the script to stop executing and terminate immediately.
            exit 0
        }
        
        # 'trap' is a command that listens for specific system signals.
        # 'INT' stands for SIGINT (Signal Interrupt. i.e.: Ctrl+C)
        # We want to say: When the user presses Ctrl+C, do not
        # immediately kill the script. Instead, execute the cleanup function first.
        trap cleanup INT
        
        cat /dev/ttyACM0 | tee -a "$LOG_FILE"
    else
        # when the user just uses --monitor without --save-log, run the following trap.
        # 1. echo "" (prints a blank line so the terminal prompt doesn't look messy after we hit Ctrl+C).
        # 2. Prints "LOG: Exiting monitor..."
        # 3. exit 0 (exits the script cleanly).
        # Without this trap, hitting Ctrl+C to exit the cat monitor would just
        # kill the script abruptly without giving any feedback to the user.
        trap 'echo ""; echo "LOG: Exiting monitor..."; exit 0' INT
        cat /dev/ttyACM0
    fi
else
    echo "LOG: Done. Use --monitor or --save-log flag next time to automatically start the monitor."
fi
