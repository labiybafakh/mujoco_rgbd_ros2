import os
from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2

project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
scene_path = os.path.join(project_dir, "scene", "env_helicopter.simscene.xml")

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.loadScene(scene_path)


# Get handles for Kinect cameras using absolute paths
depth_cam_handle = sim.getObject('/Helicopter/kinect/depth')
color_cam_handle = sim.getObject('/Helicopter/kinect/rgb')

sim.startSimulation()
while sim.getSimulationTime() < 10:
    # Get images from Kinect cameras each step
    depth_image = sim.getVisionSensorDepth(depth_cam_handle)
    color_image, [rgb_resolution_x, rgb_resolution_y] = sim.getVisionSensorImg(color_cam_handle)

    # Process color image
    color_image = np.frombuffer(color_image, dtype=np.uint8).reshape(rgb_resolution_y, rgb_resolution_x, 3)
    color_image = cv2.flip(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB), 0)
    cv2.imshow('RGB Image', color_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    sleep(0.01)
sim.stopSimulation()