# Robot SSH Guide

This document describes how to SSH into the robot running in the ROS2 workspace.

## Overview

The robot is built on a Jetson Orin Nano Super platform.

## Connection Steps

1. **WiFi Connection**:  
   - Enable the hotspot on the Jetson.  
   - Connect to the WiFi hotspot using:
     - **SSID**: vector_jetson  
     - **Password**: 12345678

2. **SSH Access**:  
   - Open your terminal and run the command:

     ```bash
     ssh vector@rover.local
     ```

   - When prompted, enter the password: **vector123**
   - If you encounter a warning about the authenticity of the host, type `yes` to continue connecting.
