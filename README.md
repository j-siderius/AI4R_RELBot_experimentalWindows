# AI4R_RELBot

**AI for Autonomous Robot** Course  
University of Twente — Assignment 1

This repository contains the source code and setup instructions for **Assignment 1** of the AI for Autonomous Robot course at the University of Twente. You will extend the RAM department’s hardware module (RELBot) with AI-driven capabilities, including object detection, tracking, and SLAM, all implemented using modern deep-learning techniques.

All development should occur within the provided Docker container. Preserve your code snippets and submit them as ROS 2 packages for evaluation.

### [Windows installation and setup instructions](#windows-instructions)

---
# ROS2 Package: `relbot_video_interface`

This skeleton package captures a GStreamer video stream and publishes object position messages to the RELBot. It exposes the video capture loop and the `/object_position` topic to control the RELBot.

---

## Prerequisites

- **Ubuntu** host (tested on 20.04+)
- **RELBot** hardware module

### Preparing the Host PC with Docker container

Before you begin, clone this repository (which includes the `Dockerfile`, the `assignment1_setup.sh` script, and example ROS2 source code) and use the provided script to build and launch your Docker environment:

```bash
# 1. Clone the repo and enter it
git clone https://github.com/UAV-Centre-ITC/AI4R_RELBot.git
cd AI4R_RELBot

# 2. Make the helper script executable
chmod +x assignment1_setup.sh

# 3. Run the script to build (first run) or attach (subsequent runs):
./assignment1_setup.sh  # use in every shell where you want a ROS2 Docker session
```

This script will:

- Build the Docker image (named by the script) on first invocation.
- Start or attach to an existing container on future runs.
- Mount your host workspace into the container at `/ros2_ws/src` by default.

The default mapping is controlled by these variables at the top of `assignment1_setup.sh`:

```bash
HOST_FOLDER="${1:-$(pwd)/ai4r_ws/src}"    # host path to mount (defaults to ./ai4r_ws/src)
CONTAINER_FOLDER="/ros2_ws/src"           # container mount point
```

Adjust those or add extra `-v <host_dir>:<container_dir>` flags if you need additional mounts. (You have to remove and start the container again to see the effect)

Inside the container, you’ll find your workspace at `/ros2_ws/src`.  All packages you create there will persist on the host.


---

## Step 1: Connect to the RELBot

1. **WLAN (default)**  
   - The RELBot auto-connects to SSID **RAM-lab**.  
   - Ensure your host PC is on **RAM-lab**.  

2. **Ethernet (alternative)**  
   - Connect a standard Ethernet cable between your Ubuntu host and the RELBot.  
   - In **Settings > Network > Wired**, set IPv4 Method to **"Shared to other computers"** (this shares your host’s connection via DHCP).  
   - Check your host’s interface and assigned IP with:
     ```
     ifconfig  # look under eth0, enp*, or similar
     ```
   - The RELBot’s Ethernet IP appears on its LCD; alternatively, view connected devices via:
     ```
     arp -n
     ```

3. **Determine IPs**  
   - **RELBot IP**: read from the LCD.  
   - **Host IP**: run `ifconfig` and note the address under your active interface (`wlan0`, `eth0`, etc.).  

4. **SSH access**  
   ```
   ssh pi@<RELBOT_IP>
   # For GUI apps, enable X11 forwarding:
   ssh -X pi@<RELBOT_IP>
   ```

> **Tip:** Open three terminals on the RELBot. VS Code’s Remote SSH extension can simplify this.

---

## Step 2: Stream External Webcam Video

The RELBot’s external webcam is typically available at `/dev/video2`. If you encounter errors or no video, list all video devices and choose the one labeled “Webcam.”

1. **List video devices**  
   ```
   v4l2-ctl --list-devices
   ```
   _Example output (Webcam section only):_
   ```
   Webcam: Webcam (usb-0000:01:00.0-1.4):
       /dev/video2
       /dev/video3
       /dev/media1
   ```
   Here, use `/dev/video2` for MJPEG streaming.

2. **Determine your host IP**  
   ```
   ifconfig  # look under wlan0 or eth0
   ```

3. **Start the GStreamer pipeline** _(replace `<HOST_IP>` with your host IP)_  
   ```
   gst-launch-1.0 -v \
   v4l2src device=/dev/video2 ! \
   image/jpeg,width=640,height=480,framerate=30/1 ! \
   jpegdec ! videoconvert ! \
   x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! \
   rtph264pay config-interval=1 pt=96 ! \
   udpsink host=<HOST_IP> port=5000
   ```
   - **v4l2src**: captures MJPEG frames.  
   - **jpegdec + videoconvert**: decode to raw video.  
   - **x264enc**: encode to H.264 with low latency.  
   - **rtph264pay**: packetize for RTP.  
   - **udpsink**: stream to your host.

---

## Step 3: Launch Controller Nodes on the RELBot

Prebuilt ROS 2 packages for the FPGA and Raspberry Pi controllers are provided with your RELBot hardware. Report hardware issues for a replacement; software bugs will be patched across all robots.

**Terminal 1:**
```
source ~/ai4r_ws/install/setup.bash
cd ~/ai4r_ws/
sudo ./build/demo/demo   # low-level motor and FPGA interface
```

**Terminal 2:**
```
source ~/ai4r_ws/install/setup.bash
ros2 launch sequence_controller sequence_controller.launch.py   # high-level state machine
```

---

## Step 4: Configure Your Docker Container

Set up your ROS 2 workspace to communicate with the RELBot via a unique domain ID.

1. **Preview the video stream**  
   ```
   gst-launch-1.0 -v \
   udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! \
   rtph264depay ! avdec_h264 ! autovideosink
   ```

2. **Set your ROS domain**  
   ```
   export ROS_DOMAIN_ID=<RELBot_ID>   # e.g., 8 for RELBot08
   ```
   Add this line to `~/.bashrc` to make it persistent.

---

###  Build and Test relbot_video_interface Package

The `relbot_video_interface` package is already included in this repository under `/ai4r_ws/src/relbot_video_interface` and is mounted into the Docker container at `/ros2_ws/src/relbot_video_interface`.

Once your Docker container is running and prerequisites (ROS 2, GStreamer, etc.) are satisfied, build and launch the test node to verify the video stream:

```bash
# From inside the container, at the workspace root:
cd /ai4r_ws

# 1) Build only the provided package:
colcon build --packages-select relbot_video_interface

# 2) Source workspace:
source install/setup.bash

# 3) Launch the video interface node:
ros2 launch relbot_video_interface video_interface.launch.py
```

You should see the live camera feed streaming and placeholder `/object_position` messages being published. If no errors occur, the setup is correct and you can proceed to add or modify detection logic.


Happy coding—and good luck with your assignment!  

---
## ROS2 Cheatsheet

Below are common ROS 2 commands and tools for development and debugging within the Docker container:

```bash
# Build and source workspace
colcon build
source /opt/ros/jazzy/setup.bash
source /ai4r_ws/install/setup.bash

# List available packages
ros2 pkg list

# Run a node directly
ros2 run relbot_video_interface video_interface

# Launch using a launch file
ros2 launch relbot_video_interface video_interface.launch.py

# Inspect running nodes and topics
ros2 node list
ros2 topic list
ros2 topic echo /object_position

# Manage node parameters
ros2 param list /video_interface
ros2 param get /video_interface gst_pipeline
ros2 param set /video_interface gst_pipeline "<new_pipeline_string>"

# Publish a test message once
ros2 topic pub /object_position geometry_msgs/msg/Point "{x: 100.0, y: 0.0, z: 0.0}" --once

# Debug with RQT tools
rqt                  # launch RQT plugin GUI
rqt_graph            # visualize topic-node graph

# View TF frames (if using tf2)
ros2 run tf2_tools view_frames.py
# opens frames.pdf showing TF tree

# Record and playback topics
ros2 bag record /object_position  <other_topics>    # start recording
ros2 bag play <bagfile>               # play back recorded data and process it offline
```
---
## VS Code Development Environment

For seamless development, you can use VS Code’s Remote extensions to work directly inside your Docker container and SSH into the RELBot:

1. **Install Extensions 
   - **Remote Development**

2. **Connect to the Docker Container**  
   - Press <kbd>F1</kbd> → **Dev-Containers: Attach to Running Container…**  
   - Select your `relbot_ai4r_assignment1` container.  
   - VS Code will reopen with the container filesystem as your workspace.

3. **SSH into the RELBot**  
   - In the **Remote Explorer** sidebar, under **Remotes(SSH/Tunnels)**, click **Configure button** to add a host entry for `pi@<RELBOT_IP>`.  
   - Press <kbd>F1</kbd> → **Remote-SSH: Connect to Host…** → select your RELBot entry.  
   - VS Code opens a new window connected to the robot, letting you inspect logs or edit files over SSH.

4. **Workflow Tips**  
   - Use split windows: one VS Code window attached to Docker (for code build & debug), another attached to RELBot (for robot-side logs & tweaks).  
   - Ensure `~/.ssh/config` has your RELBot entry for quick access:  
     ```text
     Host  <RELBot_IP>
         HostName <RELBot_IP>
         ForwardX11 yes
         User pi
     ```

---

# Windows instructions

> [!WARNING]
> Unofficial Windows installation instructions - use at your own risk!

- Tested on Windows 10 22H2 and Windows 11 24H2 (Lenovo ThinkPad P1 Gen2 workstation laptop with NVidia GPU)

## Preparing Host PC with required software

Install VcXsrv X11 viewer for Windows:

- Navigate to the Latest release from [https://github.com/marchaesen/vcxsrv/releases](https://github.com/marchaesen/vcxsrv/releases)
- Download the `vcxsrv-64.xx.x.xx.x.installer.exe` (do not select the debug or noadmin versions)
- Run the installer and install the software

Install Docker Desktop for Windows:

- Download `Docker Desktop for Windows - x86-64` from [https://docs.docker.com/desktop/setup/install/windows-install/](https://docs.docker.com/desktop/setup/install/windows-install/)
- Run the installer and install the software
- Run Docker Desktop and enable the WSL2 engine following the guide [https://docs.docker.com/desktop/features/wsl/#turn-on-docker-desktop-wsl-2](https://docs.docker.com/desktop/features/wsl/#turn-on-docker-desktop-wsl-2)

## Preparing the Docker container

- Start Docker Desktop (after starting, the application might be hidden in your taskbar, just make sure it is running in the background)
- Using File Explorer (Verkenner), navigate to the folder where you want to place the ROS2 root folder
- Open Windows PowerShell in this folder by holding [Shift] and right-clicking, then select `Open PowerShell window here`. Confirm that PowerShell is in the correct folder by looking at the path in the window. **All following commands in this section should be executed in this window!**
- Run the following commands to clone the code repository:

```powershell
git clone https://github.com/UAV-Centre-ITC/AI4R_RELBot.git
cd AI4R_RELBot
```

- Run the following commands to build the Docker container:

```powershell
# Define names and paths
$CONTAINER_NAME = "relbot_ai4r_assignment1"
$IMAGE_NAME = "gstream-ros2-jazzy-ubuntu24"
$HOST_FOLDER = (Join-Path (Get-Location) "ai4r_ws\src").Replace('\','/')
$CONTAINER_FOLDER = "/ai4r_ws/src"

# Check if image is available, otherwise build it
try {
    $null = docker image inspect $IMAGE_NAME 2>$null
    Write-Host "Image $IMAGE_NAME already exists."
} catch {
    Write-Host "Image $IMAGE_NAME does not exist. Building the image..."
    docker build -t $IMAGE_NAME .
    
    # Check if build was successful
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Failed to build image. Exiting."
        exit 1
    }
}
```

- Verify that the docker image was built correctly by opening Docker Desktop and checking if there is a "gstream-ros2-jazzy-ubuntu24" image in the Images tab

## Preparing the host PC with Firewall settings

> [!CAUTION]
> Be aware that opening ports can have unintended consequences. You are exposing more connection surfaces to the world / internet, so be very aware of the risks! Disable / delete the opened ports after use.

In order for the Gstreamer stream and ROS messages to get passed between the RelBot and ROS2 Docker container, some Firewall ports need to be opened.

- Port 5000 UDP needs to be opened to receive the Gstreamer video stream
- Ports 7400-14903 TCP & UDP need to be opened to pass and receive ROS2 commands (default port: 7400, max_port:7400 + (250 * max_ROS_DOMAIN_ID) + 3 = 7400 + (250 * 30) + 3 = 14903)

To open the Gstreamer port in the Windows Firewall, follow the steps below:

- Press [Win] + [R] to open the Run dialog, then type `wf.msc` and press [Enter]
- In the left panel, click on [Inbound Rules], then in the right panel (Actions), click on [New Rule...]
- Select Rule type "Port" and click [Next]
- Select "UDP", then select "Specific local ports" and enter `5000`, then click [Next]
- Select "Allow the connection", then click [Next]
- Select which network types this rule applies to: select Domain, Private, and Public, then click [Next]
- Enter a name like "GStreamer UDP 5000", then click [Finish]

Repeat the steps for the ROS2 ports:

- Make a new "Port" type inbound rule
- Select "UDP", then select "Specific local ports" and enter `7400-14903`, then click [Next]
- Select "Allow the connection", then click [Next]
- Select which network types this rule applies to: select Domain, Private, and Public, then click [Next]
- Enter a name like "ROS2 UDP 7400-14903", then click [Finish]
- Make another new "Port" type inbound rule
- Select "TCP", then select "Specific local ports" and enter `7400-14903`, then click [Next]
- Select "Allow the connection", then click [Next]
- Select which network types this rule applies to: select Domain, Private, and Public, then click [Next]
- Enter a name like "ROS2 TCP 7400-14903", then click [Finish]

## Starting the container

To start the ROS2 Docker container on the host, you should follow these steps at least once. After making the container with these steps once, you can also start the container from the Docker Desktop GUI or using Docker CLI.

_In these steps, Docker is setup with NVidia graphics, not tested with other graphics platforms._

- Start Docker Desktop (after starting, the application might be hidden in your taskbar, just make sure it is running in the background)
- Start XLaunch (the VcXsrv X11 application) and configure it with: _Multiple Windows_, _Start no Client_, _Native OpenGL + Disable Access Control_ and then click `Finish` to run the application (after starting, the application might be hidden in your taskbar, just make sure it is running in the background)
- Using File Explorer (Verkenner), navigate to the `AI4R_RELBOT` folder made in the previous step
- Open Windows PowerShell in this folder by holding [Shift] and right-clicking, then select `Open PowerShell window here`. Confirm that PowerShell is in the correct folder by looking at the path in the window. **All following commands in this section should be executed in this window!**
- Run the following commands to start the initial Docker container, or to attach another terminal to the running Docker container:

```powershell
# Define names and paths
$CONTAINER_NAME = "relbot_ai4r_assignment1"
$IMAGE_NAME = "gstream-ros2-jazzy-ubuntu24"
$HOST_FOLDER = (Join-Path (Get-Location) "ai4r_ws\src").Replace('\','/')
$CONTAINER_FOLDER = "/ai4r_ws/src"

# Attach or create container
$runningContainer = docker ps --filter "name=^/$CONTAINER_NAME$" --format "{{.Names}}"
if ($runningContainer -eq $CONTAINER_NAME) {
    # Found running container
    Write-Host "Attaching to running container: $CONTAINER_NAME"
    docker exec -it $CONTAINER_NAME bash
} else {
    $existingContainer = docker ps -a --filter "name=^/$CONTAINER_NAME$" --format "{{.Names}}"
    if ($existingContainer -eq $CONTAINER_NAME) {
        # Starting existing (previous) container
        Write-Host "Starting existing container: $CONTAINER_NAME"
        docker start $CONTAINER_NAME
        docker exec -it $CONTAINER_NAME bash
    } else {
        # Launching a new container
        Write-Host "Creating and launching new container: $CONTAINER_NAME"

        # Start the docker container with the following configuration:
        # Use the bridge network mode
        # Open up port 5000 UDP for Gstreamer and ports 7400-14903 UDP and TCP for ROS2
        # Resolve the X11 host by using the built-in Docker host IP
        
        docker run -it `
            --name $CONTAINER_NAME `
            --network="bridge" `
            -p 5000:5000/udp `
            -p 7400-14903:7400-14903/udp `
            -p 7400-14903:7400-14903/tcp `
            -v "${HOST_FOLDER}:${CONTAINER_FOLDER}" `
            -e DISPLAY=host.docker.internal:0.0 `
            --privileged `
            $IMAGE_NAME bash
    }
}
```

- **After launching, keep the PowerShell window open until you want to close/stop the Docker container**

## Connecting VSCode to your Docker container

Attach VSCode to the ROS2 Docker container to make working on the assignments easier. The Docker container must be running!

- Open VSCode
- Make sure the the Remote Development Extensions are installed - [https://marketplace.visualstudio.com/items/?itemName=ms-vscode-remote.vscode-remote-extensionpack](https://marketplace.visualstudio.com/items/?itemName=ms-vscode-remote.vscode-remote-extensionpack)
- Click the (Blue) double arrows in the bottom left of the VSCode window, then click `Attach to Running Container...`
- **OR** press [F1] / press [Ctrl]+[Shift]+[P] and search for `Dev Containers: Attach to Running Container...`
- Select the `relbot_ai4r_assignment1` container and let the connection establish
- _Optionally:_ If the folder is not shown in the sidebar, click [Open Folder] and select the ROS2 root folder (`../ai4r_ws/src`)
- Open a Terminal to the container by going to `Terminal > New Terminal`, the shell is automatically attached to the container

## Windows Gstreamer video pipeline

> [!NOTE]
> The default Linux Gstreamer pipeline does not work to preview the video on Windows because it does not implicitly handles H264 decoding. The ROS2 package `relbot_video_interface` does feature a working pipeline and remains unmodified.

1. **To preview the video stream from the RelBot**
   ```
   gst-launch-1.0 -v \
   udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! \
   rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
   ```
