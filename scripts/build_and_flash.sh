#!/bin/bash
# This bash script automates the following steps:
#   0- sources and activates zephyr environment (for west usage)
#   1- clean the build dir
#   2- build the software for promicro_nrf52840 using --pristine to clear cached data
#   3- convert the output hex file to uf2 file
#   4- flash the firmware
#   5- If user wants to trigger screen command, do trigger it.

# Exit immediately if a command exits with a non-zero status.
# This ensures that if a step like 'west build' fails, it won't proceed to flash.
set -e

# Variable to track whether the user want to start screen session or not.
START_SCREEN=0
# whether to source zephyr environment or not.
SOURCE_ZEPHYR=0

# Parse arguments
# "$@" is a special Bash variable that holds all the command-line arguments passed to the script
for arg in "$@"
do
    case $arg in
        --screen)
        START_SCREEN=1
        shift # This shifts all command-line arguments one position to the left (e.g., what was argument 2 becomes argument 1). It's commonly used to "consume" arguments once they've been processed.
        ;; # equavilent to 'break' in C
        -s|--source-zephyr)
        SOURCE_ZEPHYR=1
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

if [ $START_SCREEN -eq 1 ]; then
    echo "LOG: 5. Starting screen monitor on /dev/ttyACM0 at 115200 baud..."
    # Screen will take over the terminal until you detach or exit.
    screen /dev/ttyACM0 115200
else
    echo "LOG: Done. Use --screen flag to automatically start the screen monitor."
fi
