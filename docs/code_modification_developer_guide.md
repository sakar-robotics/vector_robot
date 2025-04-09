# Robot Developer Guide

1. Overview  
   - The Jetson Orin Nano acts as the main PC to process information.
   - Two workspaces are maintained:
     - **Mobile Base Workspace:** Located at `/home/sr09/vector_ws` for the ROS2 (ROS23) mobile base.
     - **Robotic Arm Workspace:** Located at `/home/sr09/manipulator_ws`.

2. Workspace Setup and Dependencies  
   - Both workspaces are sourced in the bashrc script.
   - Additional dependencies like Micro ROS and MoveIt2 (source built) have already been installed on the Jetson Orin Nano.

3. Code Management  
   - Use GitHub to push and pull code to the Jetson via SSH.
   - Alternatively, you can use VS Code SSH tools to open an editor inside the Jetson from your development PC.

4. Network Configuration for the Robotic Arm  
   - For communication with the IGUS robotic arm over Ethernet, two network profiles are used:
     - **igus_robotic_arm:** For connecting to the robotic arm.
     - **Internet:** Used to access the internet when the igus_robotic_arm profile does not support it.
   - Use the following commands to switch between the network profiles:

    ```bash
     sudo nmcli con up igus_robotic_arm
     sudo nmcli con up Internet
    ```
