#!/bin/bash

UDEV_RULE_FILE="/etc/udev/rules.d/99-ds4slim.rules"
UDEV_RULE_CONTENT='SUBSYSTEM=="input", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="09cc", MODE="0666", TAG+="uaccess"'

echo "--- Creating/Updating DualShock 4 Slim udev rule ---"

# Check if the rule file already exists
if [ -f "$UDEV_RULE_FILE" ]; then
    echo "Rule file '$UDEV_RULE_FILE' already exists. Overwriting with new content."
    echo "$UDEV_RULE_CONTENT" | sudo tee "$UDEV_RULE_FILE" > /dev/null
else
    echo "Creating new rule file '$UDEV_RULE_FILE'."
    echo "$UDEV_RULE_CONTENT" | sudo tee "$UDEV_RULE_FILE" > /dev/null
fi

# Ensure the file permissions are correct (644 is standard for udev rules)
sudo chmod 644 "$UDEV_RULE_FILE"

echo "Reloading udev rules..."
sudo udevadm control --reload-rules

echo "Triggering udev events to apply the new rules..."
sudo udevadm trigger

echo "Udev rule for DualShock 4 Slim added and applied."
echo "If you're using a container, './container restart' to apply the changes."