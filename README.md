# AI4R_RELBot

**AI for Autonomous Robot** Course  
University of Twente — Assignment 1

This repository contains the source code and setup instructions for **Assignment 1** of the AI for Autonomous Robot course at the University of Twente. You will extend the RAM department’s hardware module (RELBot) with AI-driven capabilities, including object detection, tracking, and SLAM, all implemented using modern deep-learning techniques.

All development should occur within the provided Docker container. Preserve your code snippets and submit them as ROS 2 packages for evaluation.

---

## Prerequisites

- Ubuntu host (tested on 20.04+)  
- Docker setup with ROS 2 Jazzy (installed in the container via the supplied Dockerfile and setup script)  
- RELBot hardware module

---

## Step 1: Connect to the RELBot

1. **WLAN (default)**  
   - The RELBot auto-connects to SSID **RAM-lab**.  
   - Ensure your host PC is on **RAM-lab**.  

2. **Ethernet (alternative)**  
   - Connect a standard Ethernet cable between your Ubuntu host and the RELBot.  
   - In **Settings > Network > Wired**, set IPv4 Method to **"Shared to other computers"** (this shares your host’s connection via DHCP).  
   - Check your host’s interface and assigned IP with:
     ```bash
     ifconfig  # look under eth0, enp*, or similar
     ```
   - The RELBot’s Ethernet IP appears on its LCD; alternatively, view connected devices via:
     ```bash
     arp -n
     ```

3. **Determine IPs**  
   - **RELBot IP**: read from the LCD.  
   - **Host IP**: run `ifconfig` and note the address under your active interface (`wlan0`, `eth0`, etc.).  

4. **SSH access**  
   ```bash
   ssh pi@<RELBOT_IP>
   # For GUI apps, enable X11 forwarding:
   ssh -X pi@<RELBOT_IP>
   ```

> **Tip:** Open three terminals on the RELBot. VS Code’s Remote SSH extension can simplify this.

---

## Step 2: Stream External Webcam Video

The RELBot’s external webcam is typically available at `/dev/video2`. If you encounter errors or no video, list all video devices and choose the one labeled “Webcam.”

1. **List video devices**  
   ```bash
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
   ```bash
   ifconfig  # look under wlan0 or eth0
   ```

3. **Start the GStreamer pipeline** _(replace `<HOST_IP>` with your host IP)_  
   ```bash
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
```bash
cd ai4r_ws/
source install/setup.bash
sudo ./build/demo/demo   # low-level motor and FPGA interface
```

**Terminal 2:**
```bash
source install/setup.bash
ros2 launch sequence_controller sequence_controller.launch.py   # high-level state machine
```

---

## Step 4: Configure Your Docker Container

Set up your ROS 2 workspace to communicate with the RELBot via a unique domain ID.

1. **Preview the video stream**  
   ```bash
   gst-launch-1.0 -v \
     udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! \
     rtph264depay ! avdec_h264 ! autovideosink
   ```

2. **Set your ROS domain**  
   ```bash
   export ROS_DOMAIN_ID=<RELBot_ID>   # e.g., 8 for RELBot08
   ```
   Add this line to `~/.bashrc` to make it persistent.

---

Happy coding—and good luck with your assignment!  