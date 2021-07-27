#!/bin/bash

# Remove build folder
FOLDER=build
if  [ ! -d  "$FOLDER"  ]; then
echo "Folder does not exist, please return to the root directory to execute "
else
echo "Folder exist"
sudo rm -rf $FOLDER
fi

# Get git submodules
git submodule update --init --recursive --depth 1

exit 0
