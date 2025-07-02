#!/bin/bash

# ROS 2 Image Processor Test Script
# Starts all necessary components in sequence

echo "Starting ROS 2 Image Processor Test..."
echo "========================================"

# Step 0: Check and download rosbag2 if needed
echo "Step 0: Checking for rosbag2 files..."
if [ ! -d "rosbag2_2025_06_16-15_16_29" ] || [ ! -f "rosbag2_2025_06_16-15_16_29/rosbag2_2025_06_16-15_16_29_0.db3" ]; then
    echo "Rosbag2 files not found. Running download script..."
    chmod +x download_rosbag.sh
    ./download_rosbag.sh
    if [ $? -ne 0 ]; then
        echo "Failed to download rosbag2 files"
        exit 1
    fi
else
    echo "Rosbag2 files found, proceeding..."
fi

# Step 1: Build Docker image (to reflect any code changes)
echo "Step 1: Building Docker image to reflect code changes..."
docker build -t DFL_image_processor .

if [ $? -eq 0 ]; then
    echo "Docker image built successfully"
else
    echo "Failed to build Docker image"
    exit 1
fi

# Step 2: Clean up any existing container
echo "Step 2: Cleaning up any existing container..."
docker stop ros2_image_processor 2>/dev/null || true
docker rm ros2_image_processor 2>/dev/null || true
echo "Cleanup completed"

# Step 3: Start Docker container
echo "Step 3: Starting Docker container..."
docker run --rm -d --name ros2_image_processor \
    -v $PWD/rosbag2_2025_06_16-15_16_29:/ros2_ws/rosbag2_2025_06_16-15_16_29:ro \
    -v $PWD/visualization_output:/ros2_ws/visualization_output:rw \
    -v $PWD/homography_matrix.npy:/ros2_ws/homography_matrix.npy:ro \
    study_0702-ros2_image_processor tail -f /dev/null

if [ $? -eq 0 ]; then
    echo "Container started successfully"
else
    echo "Failed to start container"
    exit 1
fi

# Step 4: Build and start image processor node
echo "Step 4: Building and starting image processor node..."
docker exec ros2_image_processor bash -c "
    source /opt/ros/humble/setup.bash && 
    cd /ros2_ws && 
    colcon build --packages-select ros2_image_processor && 
    source install/setup.bash && 
    nohup python3 src/ros2_image_processor/ros2_image_processor/image_processor_node.py > /tmp/node.log 2>&1 &
"

# Step 5: Wait for node initialization (Mask2Former model download time)
echo "Step 5: Waiting for node initialization and Mask2Former model download..."
sleep 15

# Step 6: Check if node is running
echo "Step 6: Checking node status..."
docker exec ros2_image_processor bash -c "
    source /opt/ros/humble/setup.bash && 
    source /ros2_ws/install/setup.bash && 
    timeout 5 ros2 node list
"

# Show initial logs
echo ""
echo "Initial node logs:"
docker exec ros2_image_processor tail -n 20 /tmp/node.log

# Step 7: Start ROS bag playback with controlled rate
echo ""
echo "Step 7: Starting ROS bag playback with controlled rate..."
docker exec ros2_image_processor bash -c "
    source /opt/ros/humble/setup.bash && 
    timeout 600 ros2 bag play /ros2_ws/rosbag2_2025_06_16-15_16_29 --rate 2.0
"

# Step 8: Wait a moment for processing to complete
echo "Step 8: Waiting for processing to complete..."
sleep 5

# Step 9: Show final results
echo ""
echo "Final Results:"
echo "=================="

# Check ROS topics
echo "Active ROS topics:"
docker exec ros2_image_processor bash -c "
    source /opt/ros/humble/setup.bash && 
    timeout 5 ros2 topic list
"

# Show final logs
echo ""
echo "Final node logs:"
docker exec ros2_image_processor tail -n 30 /tmp/node.log

# Check output files
echo ""
echo "Output files:"
ls -la visualization_output/stitched/ | head -5
echo "..."
ls -la visualization_output/segmented/ | head -5

echo ""
echo "Test completed!"
echo "Note: If you have a GPU, you can edit the Dockerfile to use CUDA-enabled torch for faster processing." 