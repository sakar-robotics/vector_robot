#!/bin/bash

# Script to add a command to sudoers for no-password execution

echo "Enter the command you want to run without a password (full path required):"
read -r command_path

# Validate that the command exists
if ! command -v "${command_path%% *}" &> /dev/null; then
    echo "Error: Command not found. Ensure you entered the full path."
    exit 1
fi

# Extract the username
USER_NAME=$(whoami)

# Check if the user is root
if [ "$USER_NAME" != "root" ]; then
    echo "You must run this script with sudo:"
    echo "sudo $0"
    exit 1
fi

# Add the command to sudoers
echo "$SUDO_USER ALL=(ALL) NOPASSWD: $command_path" | sudo tee -a /etc/sudoers > /dev/null

echo "Command added to sudoers: $command_path"
echo "You can now run it without a password using sudo."

# Test the command
echo "Testing the command..."
sudo $command_path

echo "Setup complete! The command should now work without a password."
