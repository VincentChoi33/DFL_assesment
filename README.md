# ROS 2 Image Processor - Technical Assessment

This project implements a ROS 2 node that processes dual camera streams from Intel RealSense cameras, performs image stitching, and implements object tracking for humans and vehicles with motion path visualization.

## 🎯 Features

- **Dual Camera Stream Processing**: Subscribes to two camera streams (`/realsense/left/color/image_raw_throttle` and `/realsense/right/color/image_raw_throttle`)
- **Image Stitching**: Creates a panoramic view using homography-based stitching
- **Semantic Segmentation**: Uses Mask2Former model for pixel-level classification
- **GPU Acceleration**: Supports CUDA for faster processing
- **Real-time Processing**: Processes and publishes results at high FPS
- **Image Saving**: Automatically saves processed and segmented images

## 📋 Prerequisites

- Docker
- NVIDIA GPU with CUDA support (for GPU version)
- ROS 2 Humble (if running natively)
- Python 3.8+
- PyTorch (with CUDA support for GPU version)
- OpenCV 4.x
- NumPy
- Transformers (Hugging Face)

## 📦 Dataset Download (rosbag2)

This project requires a large ROS 2 bag file (~2.5GB) for testing and demonstration. The bag file will be **automatically downloaded** if it is not present when you run `./start_cpu.sh` or `./start_gpu.sh`.

You can also manually download the bag file using the provided script:

```bash
./download_rosbag.sh
```

- If the `rosbag2_2025_06_16-15_16_29` directory and files already exist, the script will skip the download.
- The script checks for disk space and required tools (`curl`, `unzip`).
- If you run the start scripts, they will call this script automatically if needed.

## 🚀 Quick Start with Docker

### 1. GPU Version with Docker Compose (Recommended)

```bash
docker-compose -f docker-compose-gpu.yml up --build
```

### 2. CPU Version with Docker Compose

```bash
docker-compose -f docker-compose-cpu.yml up --build
```

### 3. GPU Version with Script

```bash
./start_gpu.sh
```

### 4. CPU Version with Script

```bash
./start_cpu.sh
```

**Note:**
- The first time you run the start scripts, the required rosbag2 file will be downloaded automatically if missing.
- You can also run `./download_rosbag.sh` manually to pre-download the dataset.

This will:
- Build the Docker image with appropriate dependencies
- Launch the ROS 2 container
- Build and start the image processor node
- Play the ROS bag file automatically
- Process images with semantic segmentation

### 3. View the Results

The processed images with semantic segmentation will be published to the `/processed_image` and `/segmented_image` topics. You can view them using:

```bash
# In another terminal
ros2 run rqt_image_view rqt_image_view
```

The processed images are also saved to:
- `visualization_output/stitched/`: Stitched images
- `visualization_output/segmented/`: Segmented images with overlays

## 🔧 Manual Setup (Without Docker)

### 1. Install Dependencies

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install Python dependencies
pip3 install opencv-python numpy opencv-contrib-python

# Install ROS 2 packages
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

### 2. Build the Workspace

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select ros2_image_processor

# Source the workspace
source install/setup.bash
```

### 3. Run the Application

```bash
# Terminal 1: Launch the image processor
ros2 launch ros2_image_processor image_processor.launch.py

# Terminal 2: Play the bag file (if not using launch file)
ros2 bag play rosbag2_2025_06_16-15_16_29
```

## 📁 Project Structure

```
study_0254/
├── src/ros2_image_processor/
│   ├── ros2_image_processor/
│   │   ├── __init__.py
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
│   ├── stitched/
│   └── segmented/
├── start_gpu.sh
├── start_cpu.sh
├── download_rosbag.sh
├── Dockerfile.gpu
├── Dockerfile
├── docker-compose-gpu.yml
├── docker-compose-cpu.yml
├── requirements.txt
├── QUICK_START.md
├── README.md
└── IMPLEMENTATION_SUMMARY.md
```

## 🎮 Usage

### Topics

**Subscribed Topics:**
- `/realsense/left/color/image_raw_throttle` (sensor_msgs/Image): Left camera stream
- `/realsense/right/color/image_raw_throttle` (sensor_msgs/Image): Right camera stream

**Published Topics:**
- `/processed_image` (sensor_msgs/Image): Stitched image
- `/segmented_image` (sensor_msgs/Image): Segmented image with overlays

### Parameters

- `left_camera_topic`: Topic name for left camera (default: `/realsense/right/color/image_raw_throttle`)
- `right_camera_topic`: Topic name for right camera (default: `/realsense/left/color/image_raw_throttle`)
- `processed_image_topic`: Topic name for processed image (default: `/processed_image`)
- `segmented_image_topic`: Topic name for segmented image (default: `/segmented_image`)

### Launch Arguments

- `bag_file`: Path to the ROS bag file (default: `rosbag2_2025_06_16-15_16_29`)

## 🔍 Semantic Segmentation Details

### Model Information
- **Model**: Mask2Former (facebook/mask2former-swin-tiny-cityscapes-semantic)
- **Framework**: PyTorch with Hugging Face Transformers
- **Classes**: 19 Cityscapes classes (road, building, car, person, etc.)
- **Input**: RGB images
- **Output**: Pixel-level class segmentation

### Processing Pipeline
- **Image Preprocessing**: BGR to RGB conversion, PIL transformation
- **Model Inference**: GPU-accelerated with CUDA support
- **Post-processing**: Official Mask2Former post-processing
- **Color Mapping**: BGR color scheme for visualization

### Performance
- **GPU Processing**: ~10x faster than CPU
- **Memory Usage**: ~2GB GPU memory for model
- **Processing Rate**: High FPS with GPU acceleration

## 🎨 Visualization Features

- **Semantic Segmentation**: Pixel-level classification with color coding
- **Class Colors**: BGR color scheme (car: blue, person: red, road: purple, etc.)
- **Overlay Visualization**: Semi-transparent segmentation overlay on original images
- **Real-time Processing**: High FPS with GPU acceleration
- **Image Saving**: Automatic saving of both stitched and segmented images

## 🐛 Troubleshooting

### Common Issues

1. **Bag File Download Issues**
   - The bag file will be downloaded automatically if missing when you run the start scripts.
   - You can also run `./download_rosbag.sh` manually.
   - Check your internet connection and available disk space (~4GB required).
   - If the download fails, re-run the script or check for error messages.

2. **Mask2Former Model Download Issues**
   - The model will be downloaded automatically on first run
   - Check internet connection for Hugging Face model download
   - Verify sufficient disk space (~2GB for model)

3. **GPU Memory Issues**
   - Ensure sufficient GPU memory (minimum 4GB recommended)
   - Use CPU version if GPU memory is insufficient
   - Check CUDA installation and compatibility

4. **No Images Displayed**
   - Verify the bag file contains the expected topics
   - Check topic names match the expected format
   - Use `ros2 topic list` to see available topics

5. **Docker GPU Issues**
   - Ensure NVIDIA Docker runtime is installed
   - Check GPU availability with `nvidia-smi`
   - Use CPU version as fallback

### Debug Commands

```bash
# List available topics
ros2 topic list

# Check topic info
ros2 topic info /realsense/left/color/image_raw_throttle

# Monitor topic messages
ros2 topic echo /processed_image
ros2 topic echo /segmented_image

# Check node status
ros2 node list
ros2 node info /image_processor_node

# Check GPU status (in container)
nvidia-smi
```

## 📊 Performance Considerations

- Processing rate: High FPS with GPU acceleration
- Memory usage: ~2GB GPU memory for model
- CPU usage: Low with GPU acceleration
- GPU acceleration: Fully implemented with CUDA support
- Model loading: ~30 seconds on first run

## 🔮 Future Improvements

- Multi-object tracking algorithms (SORT, DeepSORT)
- Instance segmentation for individual object identification
- Depth estimation from stereo cameras
- Configurable segmentation parameters
- Recording processed videos
- Real-time performance optimization
- Support for different segmentation models

## 📝 License

This project is licensed under the Apache License 2.0.

## 🤝 Contributing

Feel free to submit issues and enhancement requests!

---

**Note**: This implementation focuses on semantic segmentation using Mask2Former as the primary processing task. The code structure is modular and can be easily extended to implement object tracking, instance segmentation, or depth estimation as alternative processing options. 