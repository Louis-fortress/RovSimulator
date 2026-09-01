#!/bin/bash

# Save this as fix_gazebo_ros_headers.sh

cd ~/uuv_ws/src/Plankton

# List of header files that need fixing
HEADER_FILES=(
    "uuv_gazebo_plugins/uuv_gazebo_ros_plugins/include/uuv_gazebo_ros_plugins/ThrusterROSPlugin.h"
    "uuv_gazebo_plugins/uuv_gazebo_ros_plugins/include/uuv_gazebo_ros_plugins/UnderwaterObjectROSPlugin.h"
    "uuv_gazebo_plugins/uuv_gazebo_ros_plugins/include/uuv_gazebo_ros_plugins/FinROSPlugin.h"
    "uuv_gazebo_plugins/uuv_gazebo_ros_plugins/include/uuv_gazebo_ros_plugins/JointStatePublisher.h"
    "uuv_gazebo_plugins/uuv_gazebo_ros_plugins/include/uuv_gazebo_ros_plugins/LinearBatteryROSPlugin.h"
    "uuv_gazebo_plugins/uuv_gazebo_ros_plugins/include/uuv_gazebo_ros_plugins/CustomBatteryConsumerROSPlugin.h"
    "uuv_gazebo_plugins/uuv_gazebo_ros_plugins/include/uuv_gazebo_ros_plugins/AccelerationsTestPlugin.h"
    "uuv_world_plugins/uuv_world_ros_plugins/include/uuv_world_ros_plugins/UnderwaterCurrentROSPlugin.h"
    "uuv_world_plugins/uuv_world_ros_plugins/include/uuv_world_ros_plugins/SphericalCoordinatesROSInterfacePlugin.h"
)

for file in "${HEADER_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "Processing $file..."
        
        # Backup the file
        cp "$file" "$file.backup"
        
        # Comment out the problematic include
        sed -i 's|^#include <gazebo_ros/node.hpp>|// #include <gazebo_ros/node.hpp> // Removed for Humble compatibility|g' "$file"
        
        # Make sure rclcpp is included (add if not present)
        if ! grep -q "#include <rclcpp/rclcpp.hpp>" "$file"; then
            # Add rclcpp include after the last #include before namespace/class
            sed -i '/#include/a #include <rclcpp/rclcpp.hpp>' "$file" | head -1
        fi
        
        echo "Fixed $file"
    else
        echo "Warning: $file not found"
    fi
done

echo "All header files processed!"
