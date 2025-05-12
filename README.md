# AI4R_RELBot

**AI for Autonomous Robot** Course  
University of Twente — Assignment 1

This repository contains the source code and setup instructions for **Assignment 1** of the AI for Autonomous Robot course at the University of Twente. You will extend the RAM department’s hardware module (RELBot) with AI-driven capabilities, including object detection, tracking, and SLAM, all implemented using modern deep-learning techniques.

All development should occur within the provided Docker container. Preserve your code snippets and submit them as ROS 2 packages for evaluation.

### ~~[Windows installation and setup instructions](#windows-instructions)~~

### [VirtualBox VM installation and setup instructions](#virtualbox-vm-instructions)

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
- Some TCP and UDP ports to pass and receive ROS2 commands

### Calculating ROS2 ports

To know which TCP and UDP ports need to be opened for ROS2, the `ROS_DOMAIN_ID` is needed. This is usually equal to your RelBot number. Calculate the ports using the formula below:

- Discovery multicast port = 7400 + (250 * ROS_DOMAIN_ID)
- Discovery unicast port = 7401 + (250 * ROS_DOMAIN_ID)
- Userdata multicast port = 7402 + (250 * ROS_DOMAIN_ID)
- Userdata unicast port = 7403 + (250 * ROS_DOMAIN_ID)

For example, with `ROS_DOMAIN_ID=1`, the ports are: [7650, 7651, 7652, 7653]. In the following steps, refer to these ports by writing `<lowest_port>-<highest_port`. The example would resolve to: 7650-7653.

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
- Select "UDP", then select "Specific local ports" and enter `<lowest_port>-<highest_port`, then click [Next]
- Select "Allow the connection", then click [Next]
- Select which network types this rule applies to: select Domain, Private, and Public, then click [Next]
- Enter a name like "ROS2 UDP ports", then click [Finish]
- Make another new "Port" type inbound rule
- Select "TCP", then select "Specific local ports" and enter `<lowest_port>-<highest_port`, then click [Next]
- Select "Allow the connection", then click [Next]
- Select which network types this rule applies to: select Domain, Private, and Public, then click [Next]
- Enter a name like "ROS2 TCP ports", then click [Finish]

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
            -p <lowest_port>-<highest_port>:<lowest_port>-<highest_port>/udp `
            -p <lowest_port>-<highest_port>:<lowest_port>-<highest_port>/tcp `
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


# VirtualBox VM instructions

> [!WARNING]
> Unofficial VirtualBox VM installation instructions - use at your own risk!

This virtual machine will emulate an Ubuntu Desktop installation on your Windows pc. The ROS2 environment can be directly installed onto Ubuntu, to forego the need for Docker containers.

- Tested on Windows 10 22H2 (Lenovo ThinkPad P1 Gen2 workstation laptop with NVidia GPU)

## Preparing software

In order to run the ROS2 packages and receive the video stream on your Windows host machine using a Ubuntu virtual machine, install or download the following software:

- Download and install Oracle Virtualbox from [https://www.virtualbox.org/wiki/Downloads](https://www.virtualbox.org/wiki/Downloads)
- Download Ubuntu Desktop 24.04.x LTS from [https://ubuntu.com/download/desktop](https://ubuntu.com/download/desktop) - newer or older major version of Ubuntu might not work!

## Setting up the Virtual Machine

In the following steps, the Virtual Machine inside of Oracle VirtualBox will be set up with Ubuntu Desktop operating system.

1. Open Oracle VirtualBox and click `New` in the top toolbar to create a new virtual machine.
2. Name the virtual machine something recognisable (for example 'Ubuntu 24.04 ROS2 VM'), and choose the downloaded Ubuntu Desktop `.iso` file as the ISO Image.
3. Tick the box 'Skip unattended installation' and click `Next`.
4. In the Hardware section, give the virtual machine about 50% Base memory and 50% CPU (this can be changed later) and click `Next`.
5. In the Virtual Harddisk section, select Create a Virtual Hard Disk and give the virtual machine at least 40.00 GB of storage in order to have space to install all dependancies, then click `Next`.
6. Finally, click `Finish`.

To install Ubuntu Desktop in the virtual machine, follow these steps:

1. Start the newly made virtual machine by clicking on it in the left sidebar, then clicking `Start` in the top toolbar.
2. After powering up, the virtual machine window will open and show a selection window. Select 'Try and Install Ubuntu' using the arrow keys and press `Enter`.
3. Go through the installer and adapt the following settings: Connect to the Internet > select 'Use wired connection', What do you want to do with Ubuntu? > select 'Install Ubuntu', How would you like to install Ubuntu? > select 'Interactive installation', Which apps would you like to install to start with? > select 'Default selection', Install recommended proprietary software? > select both options and finally How do you want to install Ubuntu? > select 'Erase disk and install Ubuntu' and click 'Install'.
4. After installing, you will be presented with the log-in screen, enter the credentials chosen earlier and verify that Ubuntu Desktop is working.
5. Go to the App Center and install 'Visual Studio Code', use [this guide](https://www.ubuntubuzz.com/2024/05/how-to-install-applications-on-ubuntu-2404.html) if needed.

## Installing software on the Virtual Machine

In order to run ROS2 and the video streamer, several packages need to be installed once (at the start). Open a Terminal inside of Ubuntu Desktop and perform all following commands there:

1. `sudo apt-get update` - this might require you to enter the user password once, updates all packages in the system.
1. `sudo apt-get install -y --no-install-recommends locales curl gnupg lsb-relase git` - installs localisation and ROS prerequisites
1. `sudo apt-get install -y --no-install-recommends python3-pip build-essentials software-properties-common` - installs Python and ROS build tools
1. `sudo apt-get install -y --no-install-recommends python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav  gstreamer1.0-x` - installs Gstreamer core + plugins + tools
1. `sudo locale-gen en_US.UTF-8 && sudo update-locale LANG=en_US.UTF-8` - set the pc locale to english
1. `sudo add-apt-repository universe` - ensure that the APT repositories are added
1. `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg` - add the ROS 2 GPG key with apt
1. `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null` - add the repository to the sources list
1. `sudo apt-get update` - updates all packages in the system again with the new ROS packages
1. `sudo apt-get install -y --no-install-recommends ros-jazzy-desktop python3-colcon-common-extensions` - install ROS2 desktop
1. `pip3 install --upgrade --break-system-packages colcon-common-extensions ultralytics opencv-python numpy` - install Python libraries
1. `sudo apt-get clean && rm -rf /var/lib/apt/lists/*` - clean up apt caches
1. `git clone https://github.com/UAV-Centre-ITC/AI4R_RELBot.git` - download the required ROS2 packages and files from this repository

<details>
   <summary>If you are brave, run all commands at once</summary>

   Copy and run all commands at once in a Terminal window:

   ```
   sudo apt-get update && 
   sudo apt-get install -y --no-install-recommends locales curl gnupg lsb-relase git &&
   sudo apt-get install -y --no-install-recommends python3-pip build-essentials software-properties-common &&
   sudo apt-get install -y --no-install-recommends python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav  gstreamer1.0-x &&
   sudo locale-gen en_US.UTF-8 && sudo update-locale LANG=en_US.UTF-8 &&
   sudo add-apt-repository universe &&
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg &&
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null &&
   sudo apt-get update &&
   sudo apt-get install -y --no-install-recommends ros-jazzy-desktop python3-colcon-common-extensions &&
   pip3 install --upgrade --break-system-packages colcon-common-extensions ultralytics opencv-python numpy &&
   sudo apt-get clean && rm -rf /var/lib/apt/lists/* &&
   git clone https://github.com/UAV-Centre-ITC/AI4R_RELBot.git
   ```
</details>

## Using the Virtual Machine

To use the virtual machine to run ROS2 and receive the video stream from the RelBot, do the following:

1. Start the VirtualBox virtual machine and log in using the user password.
2. Connect the host machine (Windows pc) to the RAM-lab WiFi network (if you want to retain internet access, you can simultaneously connect to UT-ethernet - this requires registration with [LISA SRN](https://webapps.utwente.nl/srn)).
3. In the virtual machine, click on Devices > Network > Network settings in the top toolbar
4. Change the adapter settings from 'Attached to (NAT)' to 'Bridged Adapter', then change 'Name' to your WiFi card name (for example Intel Wi-Fi 6 AX200) and click `Ok / Save`.
5. In Ubuntu, click on the system icons in the rop right, then click on 'Wired' to turn off the internet connection. Click on 'Wired' to turn on the internet again, the virtual machine will now directly use your laptops WiFi card.
6. Open a Terminal and ping the RelBot IP address (it is displayed on the small screen on the RelBot), by running this command: `ping <relbot_ip>`. You should receive back some packets.
7. In the same Terminal window, run `ipconfig` and make sure that the virtual machine has an IP address in the same range as the RelBot (for example, if the RelBot IP is `192.168.0.100`, then the virtual machine should have an IP like `192.168.0.101`). Note this IP for the Gstreamer pipeline later on.
8. To connect to the right RelBot, in the same Terminal window, run `export ROS_DOMAIN_ID=<relbot_number>` with the number of the RelBot (it is on the box).
9. Open two (2) more Terminals (either with the plus-sign in the top or by opening another instance of the application). In **all Terminals**, connect to the RelBot by running `ssh pi@<relbot_ip>` and log in using the password (can be found on Canvas).
10. In one of the Terminals, run the following command:  `export ROS_DOMAIN_ID=<relbot_number>` to receive the correct commands.
11. To start the webcam stream, the FPGA motor controller and the ROS2 controller, run the following commands:
   - In Terminal 1, launch the Gstream and enter the IP address noted in step 7 for the HOST_IP: 
   ```
   gst-launch-1.0 -v \
   v4l2src device=/dev/video2 ! \
   image/jpeg,width=640,height=480,framerate=30/1 ! \
   jpegdec ! videoconvert ! \
   x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! \
   rtph264pay config-interval=1 pt=96 ! \
   udpsink host=<HOST_IP> port=5000
   ```
   - In Terminal 2, start the FPGA motor controller: 
   ```
   source ~/ai4r_ws/install/setup.bash
   cd ~/ai4r_ws/
   sudo ./build/demo/demo   # low-level motor and FPGA interface   
   ```
   - In Terminal 3, start the ROS2 controller: 
   ```
   source ~/ai4r_ws/install/setup.bash
   ros2 launch sequence_controller sequence_controller.launch.py   # high-level state machine   
   ```

The command to start the FPGA controller should start returning motor angles once the ROS2 controller is running. If this is not the case, terminate the FPGA motor controller and the ROS2 controller, then restart the FPGA motor controller and finally restart the ROS2 controller.

12. Open Visual Studio Code and navigate to the 'AI4R_RELBot' folder.
13. Modify the ROS2 package, then save the changes.
14. To run the modified ROS2 package, open a Terminal by going to Terminal > New Terminal, then run the following commands
   - `source /opt/ros/jazzy/setup.bash` - to set up the ROS environment in this terminal (this is only needed once per terminal)
   - `colcon build && source install/setup.bash` - to build and source the ROS2 packages
   - `ros2 launch relbot_video_interface video_interface.launch.py` - to start the ROS2 package