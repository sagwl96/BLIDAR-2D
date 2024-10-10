# BLIDAR 2D Simulation for Blender
BLIDAR is a 2D LIDAR simulation plugin for Blender that allows you to assign any mesh or empty object as a LIDAR sensor, customize its parameters, and generate point cloud data. The plugin also includes features like field of view (FOV) adjustment, noise simulation, and start/stop scan functionality.

# Note: This plugin has not been tested for accuracy or robotics application yet. Please use at your own discretion! 

## To - Do
- Fix the CSV export
- Allow pointcloud publishing in ROS/ROS2
- Color the pointclouds
- Add intensity data?

## Features
- Assign any mesh or empty object as a LIDAR sensor.
- Customize LIDAR parameters such as range, frequency, and number of rays.
- Simulate real-time point cloud data.
- Control noise levels in the simulated data.
- Stop and start scanning with ease.
- Adjust the field of view (FOV) to suit your scene.

## Requirements
- Blender Version: 2.80 or higher
- Dependencies: The plugin uses the following Python libraries, which are pre-installed in Blender:
  - bpy (Blender Python API)
  - bmesh (for mesh manipulation)
  - mathutils (for vector and matrix operations)
  - math and random for geometric calculations and randomness

## Installation
- Download the __blidar_simulation.py__ file from this repository.
- Open Blender and go to Edit > Preferences > Add-ons.
- Click Install, locate the blidar_simulation.py file, and install it.
- Once installed, enable the BLIDAR 2D add-on from the Add-ons list.

## Usage
### Assigning a LIDAR
- Select an empty object in your scene (do not use mesh as it will self intersect with the raycasts for lidar).
- In the Object context menu, you’ll find the BLIDAR section.
- Click Assign LIDAR to designate the selected object as the LIDAR sensor.
 
### Start Scanning
- After assigning a LIDAR, click Start Scanning to begin generating point cloud data.
- The plugin will simulate the LIDAR's behavior, creating a point cloud in real-time.
- The scan data will be automatically displayed in the scene as the scan progresses.
  
### Customizing Parameters
- Range: You can set the maximum distance the LIDAR can scan.
- FOV: Adjust the field of view to control how wide the scanning area is.
- Noise: Add noise to simulate real-world imperfections in the LIDAR sensor.
- Stop Scanning: Click Stop Scanning to halt the point cloud generation at any time.
  
### Exporting Point Cloud Data
- The plugin allows exporting point cloud data as .csv for further analysis (work in progress).

## Contributing
Contributions, bug reports, and feature requests are welcome! Feel free to fork this repository and submit pull requests.

## License
This project is licensed under the MIT License.
