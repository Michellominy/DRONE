#!/bin/bash
# setup_pid_mb.sh
# Run this script in the same directory where pid_mb.c and pid_mb/ folder are located.

# Exit on error
set -e

# MicroBlaze target folder (you can change PMODB -> PMODA, ARDUINO, etc.)
TARGET_DIR="/home/xilinx/pynq/lib/pmod/pid_mb"

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

# Go to project directory
cd "$TARGET_DIR"

echo ">>> Project setup complete!"

