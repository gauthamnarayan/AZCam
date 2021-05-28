# AZCam
A package to easily obtain a images and point clouds from the Azure Kinect camera ROS node

## Installation:
pip install -e setup.py

## Usage
```
import AZcam
# Init a new ROS node if there is no other ROS node started within your program.
# Use the namespace for your rostopic, if none leave empty. 
azcam = AZCam.AZCam(init_new_node=True, namespace='azcam_front')

# Returns image in the sensors original resolution without resizing
rgbImage = azcam.getCurrentRGBImage() 
depthImage = azcam.getCurrentDepthImage()

# Open3D pointcloud functionality
pointcloud = azcam.getCurrentPointCloud2() 
azcam.savePointCloud(filename, pointcloud)

threshPointCloud = azcam.getThresholdedPointCloud(pointcloud, threshold) 
azcam.displayPointCloud(threshPointcloud) # Display thresholded pointcloud using Open3D visualizer

# Obtain Images in 480x480 resolution
rgbImage480x480 = azcam.getCurrentRGBImage480x480()
depthImage480x480 = azcam.getCurrentDepthImage480x480()

# Video collection
azcam.startRGBVideo480x480() # To start video collection asynchronously
azcam.stopRGBVideo480x480() # To stop video collection asynchronously

# To obtain video collected
rgbVideo = azcam.getLatestRGBVideo480x480 # Actually a numpy array of frames

# To save video collected
azcam.saveLatestRGBVideo480x480(filename) # Will write out each frame separately, does not compress.

# To transform actions from camera pixels to world coordinate system
# using ROS TF transforms, the camera needs to be included in the TF Tree.
cam3DPt = getCamera3DPoint(cam2DPt)
world3DPt = getWorld3DPoint(cam3DPt)
```
