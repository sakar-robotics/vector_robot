# Setting Up Automatic Startup for ROS2 on Jetson Orin Nano

This guide explains how to set up a systemd service to run a shell script on startup. The shell script will source the ROS2 workspace and launch the necessary nodes. The setup consists of a **script** and a **service file**, which should be placed in the appropriate locations.

## 2. Copy Files to Their Respective Locations

Run the following commands to copy the necessary files to their correct locations:

```bash
# Copy the shell script to the home directory
cp ~/vector_ws/src/scripts/robot_startup.sh ~/robot_startup.sh
chmod +x ~/robot_startup.sh

# Copy the systemd service file to system directory
sudo cp ~/vector_ws/src/scripts/robot_startup.service /etc/systemd/system/robot_startup.service
```

## 3. Enable the Service

Reload systemd, enable the service to run on startup, and start it immediately:

```bash
sudo systemctl daemon-reload
sudo systemctl enable robot_startup.service
sudo systemctl start robot_startup.service
```

## 4. Verify Service Status

Check if the service is running correctly:

```bash
sudo systemctl status robot_startup.service
```

To view logs for debugging, use:

```bash
journalctl -u robot_startup.service
```

## 5. Modifying the Script

If you need to make changes to the shell script:

```bash
nano ~/robot_startup.sh
```

After modifications, restart the service:

```bash
sudo systemctl restart robot_startup.service
```

---

With this setup, the Jetson Orin Nano will automatically launch ROS2 nodes on startup.
