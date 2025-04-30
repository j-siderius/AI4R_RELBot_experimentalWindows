## AI4R_RELBot

<span style="color:red;">**Unofficial Windows installation instructions - use at your own risk**</span>

- Tested on Windows 10 22H2

### Preparing Host PC with required software

Install VcXsrv X11 viewer for Windows:

- Navigate to the Latest release from [https://github.com/marchaesen/vcxsrv/releases](https://github.com/marchaesen/vcxsrv/releases)
- Download the `vcxsrv-64.xx.x.xx.x.installer.exe` (do not select the debug or noadmin versions)
- Run the installer and install the software

Install Docker Desktop for Windows:

- Download `Docker Desktop for Windows - x86-64` from [https://docs.docker.com/desktop/setup/install/windows-install/](https://docs.docker.com/desktop/setup/install/windows-install/)
- Run the installer and install the software
- Run Docker Desktop and enable the WSL2 engine following the guide [https://docs.docker.com/desktop/features/wsl/#turn-on-docker-desktop-wsl-2](https://docs.docker.com/desktop/features/wsl/#turn-on-docker-desktop-wsl-2)

### Preparing the Docker container

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

### Starting the container

The following steps should be executed every time you want to run the ROS2 Docker container

- Start Docker Desktop (after starting, the application might be hidden in your taskbar, just make sure it is running in the background)
- Start XLaunch (the VcXsrv application) and configure it with: Multiple Windows, Start no Client, Native OpenGL + Disable Access Control and click Finish to run the application (after starting, the application might be hidden in your taskbar, just make sure it is running in the background)
- Using File Explorer (Verkenner), navigate to the `AI4R_RELBOT` folder made in the previous step
- Open Windows PowerShell in this folder by holding [Shift] and right-clicking, then select `Open PowerShell window here`. Confirm that PowerShell is in the correct folder by looking at the path in the window. **All following commands in this section should be executed in this window!**
- Run the following commands to start the initial Docker container, or to attach another terminal to the running Docker container:
```powershell
# Get host IP for X11 passthrough
$hostIP = (
    Get-NetIPAddress | 
    Where-Object { $_.AddressFamily -eq 'IPv4' -and $_.PrefixOrigin -ne 'WellKnown' -and $_.IPAddress -ne '127.0.0.1' } | 
    Select-Object -First 1
).IPAddress

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
        
        # For Windows, we set up X11 with the host IP
        docker run -it `
            --name $CONTAINER_NAME `
            -e DISPLAY="$hostIP`:0.0" `
            -e QT_X11_NO_MITSHM=1 `
            -e LIBGL_ALWAYS_INDIRECT=0 `
            -e MESA_GL_VERSION_OVERRIDE=3.3 `
            -e NVIDIA_DRIVER_CAPABILITIES=all `
            -e LIBGL_ALWAYS_SOFTWARE=1 `
            -v "${HOST_FOLDER}:${CONTAINER_FOLDER}" `
            --privileged `
            $IMAGE_NAME bash
    }
}
```
- **After launching, keep the PowerShell window open until you want to close/stop the Docker container**
- *Optional:* you can also start the container from the Docker Desktop application or Docker CLI

#### Connecting VSCode to your Docker container

Attach VSCode to the ROS2 Docker container to make working on the assignments easier. The Docker container must be running!

- Open VSCode
- Make sure the the Remote Development Extensions are installed - [https://marketplace.visualstudio.com/items/?itemName=ms-vscode-remote.vscode-remote-extensionpack](https://marketplace.visualstudio.com/items/?itemName=ms-vscode-remote.vscode-remote-extensionpack)
- Click the Blue arrows in the bottom left of the VSCode window, then click `Attach to Running Container...`
- **OR** press [F1] / press [Ctrl]+[Shift]+[P] and search for `Dev Containers: Attach to Running Container...`
- Select the `relbot_ai4r_assignment1` container and let the connection establish
- Open a Terminal to the container by going to `Terminal > New Terminal`, the shell is automatically attached to the container
