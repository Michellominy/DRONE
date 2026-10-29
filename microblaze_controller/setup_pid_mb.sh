#!/bin/bash
# setup_pid_mb.sh
# Run this script in the same directory where pid_mb/ folder is located.

# Exit on error
set -e

# MicroBlaze target folder (you can change PMODB -> PMODA, ARDUINO, etc.)
PMOD_DIR="/home/xilinx/pynq/lib/pmod"
TARGET_DIR="$PMOD_DIR/pid_mb"

echo ">>> Setting up MicroBlaze project for PYNQ-Z2"

# Check if pid_mb folder exists
if [ ! -d "pid_mb" ]; then
    echo "Error: pid_mb directory not found in $(pwd)"
    exit 1
fi

# Create destination if it doesn't exist
echo ">>> Creating destination: $TARGET_DIR"
mkdir -p "$TARGET_DIR"

# Copy sources
echo ">>> Copying sources into project..."
cp -r pid_mb/* "$TARGET_DIR/"

# Go to pmod directory
cd "$PMOD_DIR"

# Build all pmod using make
echo ">>> Building MicroBlaze firmware..."
make clean || true
make

# Confirm build
if [ -f "$TARGET_DIR/Debug/pid_mb.elf" ]; then
    echo ">>> Build successful: pid_mb.elf"
else
    echo "Error: Build failed, .elf not found."
    exit 1
fi


echo ">>> Project setup complete!"

