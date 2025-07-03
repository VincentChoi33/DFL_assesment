# 🚀 Quick Start Guide

This project is a dual Intel RealSense camera image stitching system using ROS 2.

## 📋 Prerequisites

- **Docker** (Required)
- **Docker Compose** (Required)
- **Git** (Optional)

## 🛠️ Installation & Setup

### 1. Project Download
```bash
git clone <repository-url>
cd study_0254
```

Or download and extract the zip file

### 2. Data Download (Required)
Download ROS 2 bag file and save it to `rosbag2_2025_06_16-15_16_29/` folder

### 3. Grant Execution Permissions
```bash
chmod +x start_gpu.sh
chmod +x start_cpu.sh
```

## 🎯 Quick Execution

### Method 1: GPU Version with Docker Compose (Recommended)
```bash
# Terminal 1: Start with GPU support using docker-compose
docker-compose -f docker-compose-gpu.yml up --build

# Terminal 2: Start ROS bag playback (required!)
docker exec ros2_image_processor bash -c "source /opt/ros/humble/setup.bash && ros2 bag play /ros2_ws/rosbag2_2025_06_16-15_16_29 --rate 1.0"
```

### Method 2: CPU Version with Docker Compose
```bash
# Terminal 1: Start with CPU only using docker-compose
docker-compose -f docker-compose-cpu.yml up --build

# Terminal 2: Start ROS bag playback (required!)
docker exec ros2_image_processor bash -c "source /opt/ros/humble/setup.bash && ros2 bag play /ros2_ws/rosbag2_2025_06_16-15_16_29 --rate 1.0"
```

**💡 Note**: Docker Compose requires manual rosbag playback. For automatic playback, use Methods 3-4 below.

### Method 3: GPU Version with Script
```bash
# Start with GPU support using script
./start_gpu.sh
```

### Method 4: CPU Version with Script
```bash
# Start with CPU only using script
./start_cpu.sh
```

## 📁 Project Structure

```
study_0254/
├── src/                          # ROS 2 package source
│   └── ros2_image_processor/
├── rosbag2_2025_06_16-15_16_29/  # ROS 2 bag file (required)
├── visualization_output/          # Generated image storage folder
├── start_gpu.sh                  # GPU version execution script
├── start_cpu.sh                  # CPU version execution script
├── Dockerfile.gpu                # GPU Docker image definition
├── Dockerfile                    # CPU Docker image definition
├── docker-compose-gpu.yml        # GPU Docker Compose configuration
├── docker-compose-cpu.yml        # CPU Docker Compose configuration
├── requirements.txt              # Python dependencies
├── QUICK_START.md               # Quick start guide
├── README.md                     # Detailed documentation
└── IMPLEMENTATION_SUMMARY.md     # Implementation details
```

## 🔧 Manual Execution (Advanced Users)

### 1. Docker Compose (Recommended)

```bash
# GPU version
docker-compose -f docker-compose-gpu.yml up --build

# CPU version
docker-compose -f docker-compose-cpu.yml up --build
```

### 2. Manual Docker Commands

```bash
# GPU version
docker build -t study_0702-ros2_image_processor:gpu -f Dockerfile.gpu .
docker run --rm -d --name ros2_image_processor --gpus all -v $PWD/rosbag2_2025_06_16-15_16_29:/ros2_ws/rosbag2_2025_06_16-15_16_29:ro -v $PWD/visualization_output:/ros2_ws/visualization_output:rw study_0702-ros2_image_processor:gpu

# CPU version
docker build -t study_0702-ros2_image_processor:latest -f Dockerfile .
docker run --rm -d --name ros2_image_processor -v $PWD/rosbag2_2025_06_16-15_16_29:/ros2_ws/rosbag2_2025_06_16-15_16_29:ro -v $PWD/visualization_output:/ros2_ws/visualization_output:rw study_0702-ros2_image_processor:latest
```

### 3. Node Execution
```bash
# Build and run the node
docker exec ros2_image_processor bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-select ros2_image_processor && source install/setup.bash && ros2 run ros2_image_processor image_processor_node"

# ROS bag playback
docker exec ros2_image_processor bash -c "source /opt/ros/humble/setup.bash && ros2 bag play /ros2_ws/rosbag2_2025_06_16-15_16_29 --rate 2.0"
```

## 📊 Expected Output

After execution, the following files will be generated:
- `visualization_output/processed_image_*.jpg`: Stitched images
- `visualization_output/segmented_image_*.jpg`: Segmented images with overlays
- ROS Topics: `/processed_image`, `/segmented_image`, `/realsense/left/color/image_raw_throttle`, `/realsense/right/color/image_raw_throttle`

## ⚡ Performance Comparison

### Processing Speed
| Version | Stitching FPS | Segmentation FPS | Total FPS | Processing Time (10min bag) |
|---------|---------------|------------------|-----------|----------------------------|
| **CPU** | ~15-20 | ~2-3 | ~2-3 | ~45-60 minutes |
| **GPU** | ~15-20 | ~25-30 | ~15-20 | ~15-20 minutes |

### Resource Usage
| Version | CPU Usage | RAM Usage | GPU Memory | Best For |
|---------|-----------|-----------|------------|----------|
| **CPU** | 80-90% | ~4GB | N/A | Development, testing |
| **GPU** | 20-30% | ~2GB | ~2GB | Production, real-time |

## 🐛 Troubleshooting

### Issue 1: Docker Permission Error
```bash
sudo usermod -aG docker $USER
# Logout and login again
```

### Issue 2: Port Conflict
```bash
docker stop ros2_image_processor
docker rm ros2_image_processor
```

### Issue 3: Build Failure
```bash
docker system prune -a
./build.sh
```

## 📝 Notes

- **First run**: Docker image build takes 5-10 minutes
- **Memory**: Minimum 4GB RAM recommended
- **Storage**: Minimum 2GB free space required
- **Network**: Internet connection required for Docker image download

## 🎉 Success Criteria

When successfully executed:
1. Docker container is running
2. ROS 2 nodes are active
3. Images are generated in `visualization_output/` folder
4. ROS topics are being published

---

**💡 Tip**: If problems occur, check logs with `docker logs ros2_image_processor`! 