# ROS 2 Image Processor - Technical Assessment

This project implements a ROS 2 node that processes dual camera streams from Intel RealSense cameras, performs image stitching, and implements semantic segmentation using Mask2Former.

## Features

- **Dual Camera Stream Processing**: Subscribes to two camera streams (`/realsense/left/color/image_raw_throttle` and `/realsense/right/color/image_raw_throttle`)
- **Image Stitching**: Creates a panoramic view using homography-based stitching
- **Semantic Segmentation**: Uses Mask2Former model for pixel-level classification
- **GPU Acceleration**: Supports CUDA for faster processing
- **Real-time Processing**: Processes and publishes results at high FPS
- **Image Saving**: Automatically saves processed and segmented images
- **Video Creation**: Converts processed images to MP4 videos
- **Processing Completion Detection**: Automatically detects when processing is complete
- **Enhanced Container Management**: Smart cleanup and conflict prevention

## Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04 or later (recommended)
- **RAM**: Minimum 8GB, 16GB recommended
- **Storage**: At least 10GB free space
- **GPU**: NVIDIA GPU with CUDA support (for GPU version, optional for CPU)

### Software Requirements
- **Docker**: [Install Docker](https://docs.docker.com/engine/install/ubuntu/)
- **Docker Compose**: Usually included with Docker
- **NVIDIA Docker**: [Install NVIDIA Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (for GPU version)

### Verify Installation
```bash
# Check Docker
docker --version
docker-compose --version

# Check NVIDIA Docker (for GPU version)
nvidia-smi
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

## Dataset Download

This project requires a large ROS 2 bag file (~2.5GB) for testing and demonstration. The bag file will be automatically downloaded if it is not present when you run the start scripts.

You can also manually download the bag file:

```bash
chmod +x download_rosbag.sh
./download_rosbag.sh
```

The script checks for existing files and skips download if already present.

## Quick Start

### Option 1: Docker Compose (Recommended)

**GPU Version:**
```bash
docker-compose -f docker-compose-gpu.yml up --build
```

**CPU Version:**
```bash
docker-compose -f docker-compose-cpu.yml up --build
```

**Important Note for Docker Compose:**
When using Docker Compose, you need to manually start the ROS bag playback in a separate terminal:

```bash
# Terminal 1: Start the container (already done above)
docker-compose -f docker-compose-gpu.yml up --build

# Terminal 2: Start ROS bag playback
docker exec ros2_image_processor bash -c "source /opt/ros/humble/setup.bash && ros2 bag play /ros2_ws/rosbag2_2025_06_16-15_16_29 --rate 1.0"
```

**Alternative: Use Start Scripts (Recommended)**
The start scripts provide automatic rosbag playback with proper cleanup:
```bash
./start_gpu.sh    # GPU version with auto rosbag playback and cleanup
./start_cpu.sh    # CPU version with auto rosbag playback and cleanup
```

**Key Features of Start Scripts:**
- **Automatic Cleanup**: Stops container and ROS bag processes when complete
- **Signal Handling**: Graceful shutdown with Ctrl+C
- **Process Monitoring**: Tracks ROS bag PID for proper termination
- **Completion Detection**: Automatically detects when processing is finished

### Option 2: Start Scripts

**GPU Version (`start_gpu.sh`):**
- Builds Docker image with CUDA support
- Downloads rosbag2 file if missing
- Starts container with GPU access
- Builds and runs the image processor node
- Plays the ROS bag file automatically
- Processes images with semantic segmentation

```bash
chmod +x start_gpu.sh
./start_gpu.sh
```

**CPU Version (`start_cpu.sh`):**
- Builds Docker image without CUDA
- Downloads rosbag2 file if missing
- Starts container without GPU access
- Builds and runs the image processor node
- Plays the ROS bag file automatically
- Processes images with semantic segmentation (slower than GPU)

```bash
chmod +x start_cpu.sh
./start_cpu.sh
```

### What to Expect

When you run the scripts, you should see output like:
```
Starting ROS 2 Image Processor Test...
Step 0: Checking for rosbag2 files...
Rosbag2 files found, proceeding...
Step 1: Building Docker image...
Step 2: Cleaning up any existing container...
Step 3: Starting Docker container...
Step 4: Building and starting image processor node...
Step 5: Waiting for node initialization...
Step 6: Checking node status...
/image_processor_node
Step 7: Starting ROS bag playback...
```

The process will take several minutes to complete. You'll see logs showing:
- Image processing progress
- Segmentation results
- File saving operations
- **Processing completion detection**: Automatic monitoring of file count changes
- **Real-time progress updates**: Live count of processed images

### View Results

The processed images are published to:
- `/processed_image`: Stitched panoramic image
- `/segmented_image`: Segmented image with overlays

**Option 1: View in Docker container**
```bash
# In a new terminal
docker exec -it ros2_image_processor bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

**Option 2: View saved images**
```bash
# Check saved images
ls -la visualization_output/stitched/
ls -la visualization_output/segmented/
```

Images are saved to:
- `visualization_output/stitched/`: Stitched images
- `visualization_output/segmented/`: Segmented images with overlays

## Video Creation

After image processing is complete, you can create MP4 videos from the saved images:

```bash
# Create videos from processed images
./create_video.sh
```

This script will create:
- `visualization_output/videos/stitched_video.mp4`: Stitched image sequence
- `visualization_output/videos/segmented_video.mp4`: Segmented image sequence  
- `visualization_output/videos/comparison_video.mp4`: Side-by-side comparison

**Video Settings:**
- **FPS**: 11.65 (147 seconds for 1712 images)
- **Format**: MP4 with H.264 encoding
- **Quality**: High quality with reasonable file size

**Requirements:**
- Docker container must be running
- ffmpeg will be automatically installed in the container if needed

## Manual Setup (Without Docker)

### Install Dependencies

```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop

# Install Python dependencies
pip3 install opencv-python numpy opencv-contrib-python torch torchvision transformers

# Install ROS 2 packages
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

### Build and Run

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select ros2_image_processor

# Source the workspace
source install/setup.bash

# Terminal 1: Launch the image processor
ros2 launch ros2_image_processor image_processor.launch.py

# Terminal 2: Play the bag file
ros2 bag play rosbag2_2025_06_16-15_16_29
```

## Project Structure

```
DFL_fin/
├── src/ros2_image_processor/
│   ├── ros2_image_processor/
│   │   ├── image_processor_node.py
│   │   ├── seg.py
│   │   └── stitching_utils.py
│   ├── launch/
│   ├── config/
│   ├── resource/
│   ├── package.xml
│   ├── setup.py
│   └── CMakeLists.txt
├── rosbag2_2025_06_16-15_16_29/
├── visualization_output/
│   ├── stitched/          # Stitched images
│   ├── segmented/         # Segmented images
│   └── videos/           # Generated MP4 videos
├── start_gpu.sh          # GPU version with auto-completion detection
├── start_cpu.sh          # CPU version with auto-completion detection
├── create_video.sh       # Video creation script
├── download_rosbag.sh    # Data download script
├── Dockerfile.gpu
├── Dockerfile
├── docker-compose-gpu.yml
├── docker-compose-cpu.yml
├── requirements.txt
├── README.md
├── QUICK_START.md
└── TECHNICAL_DECISIONS.md
```

## Topics

**Subscribed Topics:**
- `/realsense/left/color/image_raw_throttle` (sensor_msgs/Image): Left camera stream
- `/realsense/right/color/image_raw_throttle` (sensor_msgs/Image): Right camera stream

**Published Topics:**
- `/processed_image` (sensor_msgs/Image): Stitched image
- `/segmented_image` (sensor_msgs/Image): Segmented image with overlays

## Semantic Segmentation

- **Model**: Mask2Former (facebook/mask2former-swin-tiny-cityscapes-semantic)
- **Classes**: 19 Cityscapes classes (road, building, car, person, etc.)
- **Performance**: ~10x faster with GPU acceleration
- **Memory**: ~190MB GPU memory for model

## Troubleshooting

### Common Issues

1. **Permission Denied Errors**
   ```bash
   chmod +x *.sh
   ```

2. **Directory Permission Issues**
   If you encounter "Permission denied" errors when creating directories:
   ```bash
   # Set proper permissions for output directories
   mkdir -p visualization_output/stitched visualization_output/segmented visualization_output/videos
   chmod -R 777 visualization_output
   ```

2. **Bag File Download Issues**
   - Check internet connection and disk space (~4GB required)
   - Run `./download_rosbag.sh` manually if automatic download fails
   - Ensure curl and unzip are installed: `sudo apt install curl unzip`

3. **Docker Build Failures**
   - Check Docker is running: `sudo systemctl start docker`
   - Ensure you have sufficient disk space
   - Try building without cache: `docker build --no-cache -t ...`

4. **Mask2Former Model Download Issues**
   - Model downloads automatically on first run (~190MB)
   - Check internet connection and disk space
   - First run may take 5-10 minutes to download model

5. **GPU Memory Issues**
   - Ensure sufficient GPU memory (minimum 4GB recommended)
   - Use CPU version if GPU memory is insufficient
   - Check CUDA installation: `nvidia-smi`

6. **No Images Displayed**
   - Verify bag file contains expected topics
   - Use `ros2 topic list` to see available topics
   - Check if node is running: `ros2 node list`

### Debug Commands

```bash
# List available topics
ros2 topic list

# Check node status
ros2 node list
ros2 node info /image_processor_node

# Check GPU status (in container)
docker exec ros2_image_processor nvidia-smi

# View container logs
docker logs ros2_image_processor

# Check if bag file is valid
ros2 bag info rosbag2_2025_06_16-15_16_29
```

### Expected Output Files

After successful execution, you should see:
```
visualization_output/
├── stitched/
│   ├── processed_image_20250101_120000_0001.jpg
│   ├── processed_image_20250101_120000_0002.jpg
│   └── ...
└── segmented/
    ├── segmented_image_20250101_120000_0001.jpg
    ├── segmented_image_20250101_120000_0002.jpg
    └── ...
```

## Performance Comparison: CPU vs GPU

### Processing Speed (FPS)

| Version | Image Stitching | Semantic Segmentation | Total Processing | Processing Time (2min 27s bag) | Success Rate | Notes |
|---------|----------------|---------------------|------------------|----------------------------|--------------|-------|
| **CPU** | ~15-20 FPS | ~2-3 FPS | ~2-3 FPS | ~10-15 minutes | Varies by CPU | Limited by CPU computation |
| **RTX 3090** | ~15-20 FPS | ~25-30 FPS | ~15-20 FPS | ~3-5 minutes | ~1500/1712 pairs (~87.6%) | GPU-accelerated segmentation |

### Processing Completion Detection

Both CPU and GPU versions now include automatic completion detection:
- **Real-time monitoring**: Tracks file count changes every 5 seconds
- **Stability check**: Confirms completion after 3 consecutive stable readings
- **Maximum wait time**: 2 minutes before proceeding
- **Progress display**: Shows live counts of processed images

### Resource Usage

| Version | CPU Usage | Memory Usage | GPU Memory | Model Loading Time |
|---------|-----------|--------------|------------|-------------------|
| **CPU** | 80-90% | ~4GB RAM | N/A | ~30 seconds |
| **GPU** | 20-30% | ~190MB RAM | ~2GB VRAM | ~30 seconds |

### Performance Characteristics

**CPU Version:**
- **Pros**: No GPU required, works on any system
- **Cons**: Slow segmentation (~2-3 FPS), high CPU usage
- **Best for**: Development, testing, systems without GPU

**GPU Version:**
- **Pros**: Fast segmentation (~25-30 FPS), low CPU usage
- **Cons**: Requires NVIDIA GPU with CUDA
- **Best for**: Production, real-time applications

### Real-world Performance

**Processing the full rosbag2 file (2 minutes 27 seconds of data, 1712 image pairs):**
- **RTX 3090 (GPU)**: ~1500 pairs processed out of 1712 total pairs (~87.6% success rate)
- **CPU**: Processing time and success rate vary significantly based on CPU performance

**Success Rate Analysis:**
- **GPU Version**: Higher success rate due to faster processing, reducing buffer overflow
- **CPU Version**: Lower success rate due to slower processing, leading to more dropped frames
- **Buffer Management**: Both versions use 100-frame buffers with automatic cleanup

**Real-time processing capability:**
- **CPU**: Not suitable for real-time (2-3 FPS)
- **GPU**: Suitable for real-time (15-20 FPS)

### Performance Optimization

The system uses several optimization techniques:
- **Configurable processing rate**: 100 FPS timer prevents overwhelming the system
- **Thread-safe image buffers**: Concurrent processing of left/right camera streams
- **Automatic memory cleanup**: Prevents memory leaks during long sessions
- **Lazy model loading**: Only loads segmentation model when enabled
- **Smart container management**: Automatic cleanup of conflicting containers
- **Processing completion detection**: Prevents premature termination

## Future Improvements

- Multi-object tracking algorithms (SORT, DeepSORT)
- Instance segmentation for individual object identification
- Depth estimation from stereo cameras
- Configurable segmentation parameters
- Real-time performance optimization
- **Video streaming**: Real-time video output during processing
- **Batch processing**: Support for multiple rosbag files
- **Custom video formats**: Support for different video codecs and resolutions

## Technical Decisions

For detailed explanations of the technical decisions and implementation rationale, see [TECHNICAL_DECISIONS.md](TECHNICAL_DECISIONS.md).

This document covers:
- Why semantic segmentation was chosen over object tracking or depth estimation
- Image stitching method selection and implementation
- Threading model and concurrency handling
- Docker architecture decisions
- Performance optimization strategies
- Error handling and robustness considerations

## License

This project is licensed under the Apache License 2.0. 
