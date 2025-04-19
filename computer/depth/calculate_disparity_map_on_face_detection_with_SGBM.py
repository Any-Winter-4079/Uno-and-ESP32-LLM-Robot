"""
Stereo Vision Depth Testing Script
Tests depth perception using two ESP32-CAMs for stereo vision
Features:
- Face-triggered disparity map calculation
- Real-time stereo parameter adjustment
- Camera configuration management
- Image rectification and 3D reprojection
- Performance monitoring
"""

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import bootstrap

import cv2
import time
import numpy as np
import mediapipe as mp
from calibration.store_images_to_calibrate import update_camera_config
from depth.calculate_depth_with_depth_anything import get_stereo_images, rectify_left_image, rectify_right_image

# Network and camera configuration
USE_HOTSPOT = True
RIGHT_EYE_IP = "172.20.10.10" if USE_HOTSPOT else "192.168.1.180"
LEFT_EYE_IP = "172.20.10.11" if USE_HOTSPOT else "192.168.1.181"
STREAM_TIMEOUT = 3  # seconds

# Camera settings
JPEG_QUALITY = 12                # 0-63, lower means higher quality
FRAME_SIZE = "FRAMESIZE_VGA"     # 640x480 resolution

# Stereo vision parameters
STEREO_BLOCK_SIZE = 11          # Must be odd
MIN_DISPARITY = 0
NUM_DISPARITIES = 5 * 16        # Must be divisible by 16
SPECKLE_WINDOW_SIZE = 0
SPECKLE_RANGE = 2
MODE = cv2.STEREO_SGBM_MODE_HH
UNIQUENESS_RATIO = 0
PRE_FILTER_CAP = 0
DISP12MAX_DIFF = 32

# Camera endpoints
esp32_right_image_url = f"http://{RIGHT_EYE_IP}/image.jpg"
esp32_left_image_url = f"http://{LEFT_EYE_IP}/image.jpg"
esp32_left_config_url = f"http://{LEFT_EYE_IP}/camera_config"
esp32_right_config_url = f"http://{RIGHT_EYE_IP}/camera_config"

# Load stereo calibration maps
stereo_maps_dir = '../undistortion_and_rectification/stereo_maps'
stereoMapL_x = np.load(os.path.join(stereo_maps_dir, 'stereoMapL_x.npy'))
stereoMapL_y = np.load(os.path.join(stereo_maps_dir, 'stereoMapL_y.npy'))
stereoMapR_x = np.load(os.path.join(stereo_maps_dir, 'stereoMapR_x.npy'))
stereoMapR_y = np.load(os.path.join(stereo_maps_dir, 'stereoMapR_y.npy'))
Q = np.load(os.path.join(stereo_maps_dir, 'Q.npy'))

# Initialize stereo matcher with default parameters
stereo = cv2.StereoSGBM_create(
   minDisparity=MIN_DISPARITY,
   numDisparities=NUM_DISPARITIES,
   blockSize=STEREO_BLOCK_SIZE,
   P1=8 * STEREO_BLOCK_SIZE**2,
   P2=32 * STEREO_BLOCK_SIZE**2,
   disp12MaxDiff=DISP12MAX_DIFF,
   preFilterCap=PRE_FILTER_CAP,
   uniquenessRatio=UNIQUENESS_RATIO,
   speckleWindowSize=SPECKLE_WINDOW_SIZE,
   speckleRange=SPECKLE_RANGE,
   mode=MODE
)

# Initialize MediaPipe face detector
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.5)

# Trackbar callback functions for real-time parameter adjustment
def on_min_disparity_change(val):
   global stereo
   stereo.setMinDisparity(val)

def on_num_disparities_change(val):
   global stereo
   stereo.setNumDisparities(max(16, (val // 16) * 16))

def on_block_size_change(val):
   global stereo
   stereo.setBlockSize(val if val % 2 == 1 else val + 1)

def on_speckle_window_size_change(val):
   global stereo
   stereo.setSpeckleWindowSize(val)

def on_speckle_range_change(val):
   global stereo
   stereo.setSpeckleRange(val)

def on_mode_change(val):
   global stereo
   stereo.setMode(cv2.STEREO_SGBM_MODE_HH if val == 0 else cv2.STEREO_SGBM_MODE_SGBM_3WAY)

def on_uniqueness_ratio_change(val):
   global stereo
   stereo.setUniquenessRatio(val)

def on_pre_filter_cap_change(val):
   global stereo
   stereo.setPreFilterCap(val)

def on_disp12max_diff_change(val):
   global stereo
   stereo.setDisp12MaxDiff(val)

# Create window and trackbars for parameter adjustment
cv2.namedWindow("Disparity map")
cv2.createTrackbar("Min Disp.", "Disparity map", MIN_DISPARITY, 32, on_min_disparity_change)
cv2.createTrackbar("Num Disp.", "Disparity map", NUM_DISPARITIES, 16 * 10, on_num_disparities_change)
cv2.createTrackbar("Block Size", "Disparity map", STEREO_BLOCK_SIZE, 13, on_block_size_change)
cv2.createTrackbar("Speckle Win", "Disparity map", SPECKLE_WINDOW_SIZE, 200, on_speckle_window_size_change)
cv2.createTrackbar("Speckle Range", "Disparity map", SPECKLE_RANGE, 100, on_speckle_range_change)
cv2.createTrackbar("Mode", "Disparity map", 0, 1, on_mode_change)
cv2.createTrackbar("Uniq. Ratio", "Disparity map", UNIQUENESS_RATIO, 60, on_uniqueness_ratio_change)
cv2.createTrackbar("Pre Filter Cap", "Disparity map", PRE_FILTER_CAP, 100, on_pre_filter_cap_change)
cv2.createTrackbar("Disp12MaxDiff", "Disparity map", DISP12MAX_DIFF, 60, on_disp12max_diff_change)

def get_face_centroid(image):
   """
   Detects face in image and returns its centroid coordinates
   
   Args:
       image: Input image (BGR format)
   
   Returns:
       tuple: (x, y) coordinates of face centroid or None if no face detected
   """
   # Convert to RGB for MediaPipe
   image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
   results = face_detection.process(image_rgb)
   
   if results.detections:
       # Use first detected face's bounding box
       detection = results.detections[0]
       bboxC = detection.location_data.relative_bounding_box
       x, y, w, h = bboxC.xmin, bboxC.ymin, bboxC.width, bboxC.height
       return (x + w / 2, y + h / 2)
   return None

def calculate_disparity_maps(stereo, left_img_rectified, right_img_rectified):
   """
   Computes disparity map and 3D points from rectified stereo pair
   
   Args:
       stereo: StereoSGBM matcher object
       left_img_rectified: Rectified left image
       right_img_rectified: Rectified right image
   
   Returns:
       tuple: (normalized_disparity_map, 3D_points)
   """
   # Compute disparity map
   disparity = stereo.compute(left_img_rectified, right_img_rectified)
   
   # Normalize for visualization
   disp_norm = cv2.normalize(disparity, None, alpha=0, beta=255, 
                           norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
   
   # Calculate 3D coordinates
   points_3D = cv2.reprojectImageTo3D(disparity, Q)
   
   return disp_norm, points_3D

def main():
   """
   Main execution loop:
   1. Configures cameras
   2. Captures stereo images
   3. Detects faces
   4. Computes disparity maps when faces detected
   5. Tracks performance metrics
   """
   # Performance tracking
   total_face_time = 0
   total_disparity_map_time = 0
   face_iterations = 0
   disparity_map_iterations = 0
   
   # Stream state tracking
   stream_to_recover = False
   stream_active = False

   # Initialize camera configurations
   update_camera_config(esp32_left_config_url, JPEG_QUALITY, FRAME_SIZE)
   update_camera_config(esp32_right_config_url, JPEG_QUALITY, FRAME_SIZE)

   while True:
       # Handle stream recovery if needed
       if stream_to_recover and stream_active:
           print("Stream is being recovered.")
           update_camera_config(esp32_left_config_url, JPEG_QUALITY, FRAME_SIZE)
           update_camera_config(esp32_right_config_url, JPEG_QUALITY, FRAME_SIZE)
           stream_to_recover = False
           stream_active = False
       
       # Capture and process stereo images
       img_left, img_right = get_stereo_images(esp32_left_image_url, esp32_right_image_url)

       if img_left is not None and img_right is not None:
           stream_active = True
           img_left_rectified = rectify_left_image(img_left)
           img_right_rectified = rectify_right_image(img_right)

           # Detect faces and measure timing
           face_start = time.time()
           left_centroid = get_face_centroid(img_left_rectified)
           right_centroid = get_face_centroid(img_right_rectified)
           face_end = time.time()
           face_iterations += 1
           total_face_time += (face_end - face_start)

           # Calculate disparity if faces detected
           if left_centroid is not None and right_centroid is not None:
               disparity_start = time.time()
               disp_norm, points_3D = calculate_disparity_maps(
                   stereo, img_left_rectified, img_right_rectified)
               disparity_end = time.time()
               disparity_map_iterations += 1
               total_disparity_map_time += (disparity_end - disparity_start)
               
               # Display disparity map
               cv2.imshow("Disparity map", disp_norm)
               if cv2.waitKey(50) & 0xFF == ord('q'):
                   break
       else:
           print("One or both images are None.")
           stream_to_recover = True
           cv2.waitKey(1000)
           continue

   # Report performance metrics
   average_face_time = total_face_time / face_iterations
   average_disparity_map_time = total_disparity_map_time / disparity_map_iterations
   print(f"\nAverage face centroid calculation time over {face_iterations} iterations: {average_face_time:.3f} seconds")
   print(f"Average disparity map calculation time over {disparity_map_iterations} iterations: {average_disparity_map_time:.3f} seconds")

if __name__ == "__main__":
   main()