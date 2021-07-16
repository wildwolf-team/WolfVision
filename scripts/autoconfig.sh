#!/bin/bash

# Check if runs in the source dir
if [ ! -d "scripts" ]; then
    echo "ERROR: Please run autoconfig.sh in project source dir."
    exit 1
fi

# Check if runs by root user
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run autoconfig.sh by root user (sudo)."
    exit 1
fi

# Get git submodules
git submodule update --init --recursive --depth 1

# Enable pre-commit cpplint
chmod +x scripts/cpplint-pre-commit.sh
ln -sf "$(pwd)/scripts/cpplint-pre-commit.sh" ".git/hooks/pre-commit"

# Configure mindvision sdk usb connect rules
sudo ln -sf "$(pwd)/3rdparty/mindvision/linux/88-mvusb.rules" "/etc/udev/rules.d/"

# Update source
sudo apt update

# Install dependicies
cat dependencies.txt | xargs sudo apt install -y

exit 0
