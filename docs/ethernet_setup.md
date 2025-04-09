# Ethernet Configuration for IGUS Robotic Arm

## Overview

This document provides instructions to configure the Ethernet connection on a Jetson Orin Nano for the IGUS robotic arm. The setup assigns a static IP to the Ethernet interface and ensures it is persistent across reboots.

## Prerequisites

- NetworkManager installed (`nmcli` should be available by default)
- Ethernet interface identified (check with `ip link show`)

## Setup Instructions

### 1. Download and Run the Setup Script

Save the provided shell script (`configure_ethernet.sh`) to your Jetson device, then make it executable and run it:

```bash
chmod +x configure_ethernet.sh
sudo ./configure_ethernet.sh
```

This script will:

- Configure the Ethernet interface `enP8p1s0` with a static IP `192.168.3.10`.
- Set up the gateway and DNS servers.
- Apply and activate the settings.

### 2. Verify Configuration

After running the script, confirm that the settings have been applied:

```bash
nmcli con show igus_robotic_arm
ip a show enP8p1s0
```

You should see the assigned IP address `192.168.3.10` under the Ethernet interface.

## Debugging & Troubleshooting

### 1. Check Connection Status

To check if the Ethernet connection is active:

```bash
nmcli con show --active
```

### 2. Restart the Network Connection

```bash
nmcli con down igus_robotic_arm
nmcli con up igus_robotic_arm
```

### 3. Check Network Interface Details

```bash
ip addr show enP8p1s0
```

### 4. Restart NetworkManager (if needed)

```bash
sudo systemctl restart NetworkManager
```

### 5. Reapply the Script

If network settings are not persisting, rerun the script:

```bash
sudo ./configure_ethernet.sh
```

## Additional Notes

- Ensure that no other connections are conflicting with `igus_robotic_arm`.
- The interface name (`enP8p1s0`) may differ depending on your Jetson; check with `ip link show`.
- If using a different gateway, update the `configure_ethernet.sh` script accordingly.
