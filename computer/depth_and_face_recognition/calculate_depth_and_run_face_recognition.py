"""
Integrated Stereo Vision System
Combines face recognition, object detection, and depth estimation
Features:
- Real-time face recognition with DeepFace
- Object detection with YOLOv8
- Stereo depth mapping with adjustable parameters
- Camera configuration management
- Image rectification
- Automatic stream recovery with failover
- Dynamic visualization with depth information
- Multi-threaded image acquisition
"""

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import bootstrap

import cv2
import threading
import numpy as np
import urllib.request
from ultralytics import YOLO
from deepface import DeepFace
from calibration.store_images_to_calibrate import update_camera_config
from face_recognition.run_face_recognition import get_top_predictions, draw_boxes_and_labels
from depth.calculate_disparity_map_on_face_detection_with_SGBM import calculate_disparity_maps, on_min_disparity_change, \
on_num_disparities_change, on_block_size_change, on_speckle_window_size_change, on_speckle_range_change, \
on_mode_change, on_uniqueness_ratio_change, on_pre_filter_cap_change, on_disp12max_diff_change, stereo
from depth.calculate_depth_with_depth_anything import rectify_right_image, rectify_left_image

# Configuration
JPEG_QUALITY = 12                # 0-63 lower means higher quality
FRAME_SIZE = "FRAMESIZE_VGA"     # 640x480 resolution
USE_HOTSPOT = True
RIGHT_EYE_IP = "172.20.10.10" if USE_HOTSPOT else "192.168.1.180"
LEFT_EYE_IP = "172.20.10.11" if USE_HOTSPOT else "192.168.1.181"
STREAM_TIMEOUT = 3               # seconds

# Stereo vision parameters
STEREO_MAPS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                              'undistortion_and_rectification/stereo_maps')
ALLOWED_DEPTH = 0.875
STEREO_BLOCK_SIZE = 11           # Must be odd
MIN_DISPARITY = 8
NUM_DISPARITIES = 5 * 16         # Must be non-zero, divisible by 16
SPECKLE_WINDOW_SIZE = 0
SPECKLE_RANGE = 2
MODE = cv2.STEREO_SGBM_MODE_HH
UNIQUENESS_RATIO = 0
PRE_FILTER_CAP = 0
DISP12MAX_DIFF = 32

COCO_NAMES_DIR = os.path.dirname(os.path.abspath(__file__))
LABELS = ["bottle"]              # Object labels to track

# Face recognition settings
DATABASE_PATH = "production_database"
DISTANCE_METRIC = "cosine"       # Options: cosine, euclidean, euclidean_l2
BACKEND = "fastmtcnn"            # Detection backend
MODEL = "VGG-Face"               # Recognition model
THRESHOLD = 0.625                # Recognition threshold

# Note different metrics require different thresholds
# The defaults are:
# | Model       | Cosine | Euclidean | Euclidean L2 |
# |-------------|--------|-----------|--------------|
# | VGG-Face    | 0.68   | 1.17      | 1.17         |
# | Facenet     | 0.40   | 10        | 0.80         |
# | Facenet512  | 0.30   | 23.56     | 1.04         |
# | ArcFace     | 0.68   | 4.15      | 1.13         |
# | Dlib        | 0.07   | 0.6       | 0.4          |
# | SFace       | 0.593  | 10.734    | 1.055        |
# | OpenFace    | 0.10   | 0.55      | 0.55         |
# | DeepFace    | 0.23   | 64        | 0.64         |
# | DeepID      | 0.015  | 45        | 0.17         |

# For my case: 0.625, cosine, fastmtcnn, and VGG-Face worked well but you should experiment with different combinations

# Camera endpoints
esp32_right_image_url = f"http://{RIGHT_EYE_IP}/image.jpg"
esp32_left_image_url = f"http://{LEFT_EYE_IP}/image.jpg"
esp32_left_config_url = f"http://{LEFT_EYE_IP}/camera_config"
esp32_right_config_url = f"http://{RIGHT_EYE_IP}/camera_config"

# Load stereo calibration maps
stereoMapL_x = np.load(os.path.join(STEREO_MAPS_DIR, 'stereoMapL_x.npy'))
stereoMapL_y = np.load(os.path.join(STEREO_MAPS_DIR, 'stereoMapL_y.npy'))
stereoMapR_x = np.load(os.path.join(STEREO_MAPS_DIR, 'stereoMapR_x.npy'))
stereoMapR_y = np.load(os.path.join(STEREO_MAPS_DIR, 'stereoMapR_y.npy'))
Q = np.load(os.path.join(STEREO_MAPS_DIR, 'Q.npy'))

# Initialize object detection
object_detection_model = YOLO("yolov8n.pt")

# Load object class names
with open(os.path.join(COCO_NAMES_DIR, 'coco.names'), "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Create window and trackbars for stereo parameter adjustment
cv2.namedWindow("Disparity map")
cv2.createTrackbar("Min Disp.", "Disparity map", MIN_DISPARITY, 32, on_min_disparity_change)
cv2.createTrackbar("Num Disp.", "Disparity map", NUM_DISPARITIES, 16 * 16, on_num_disparities_change)
cv2.createTrackbar("Block Size", "Disparity map", STEREO_BLOCK_SIZE, 13, on_block_size_change)
cv2.createTrackbar("Speckle Win", "Disparity map", SPECKLE_WINDOW_SIZE, 200, on_speckle_window_size_change)
cv2.createTrackbar("Speckle Range", "Disparity map", SPECKLE_RANGE, 100, on_speckle_range_change)
cv2.createTrackbar("Mode", "Disparity map", 0, 1, on_mode_change)
cv2.createTrackbar("Uniq. Ratio", "Disparity map", UNIQUENESS_RATIO, 60, on_uniqueness_ratio_change)
cv2.createTrackbar("Pre Filter Cap", "Disparity map", PRE_FILTER_CAP, 100, on_pre_filter_cap_change)
cv2.createTrackbar("Disp12MaxDiff", "Disparity map", DISP12MAX_DIFF, 60, on_disp12max_diff_change)

def process_predictions(top_predictions):
    """
    Processes predictions to extract unique individuals
    
    Args:
        top_predictions: List of top predictions for each face
    
    Returns:
        dict: Dictionary of unique individuals with their info
    """
    unique_individuals = {}
    for prediction in top_predictions:
        # Extract person name from identity path
        identity_path = prediction['identity'].replace(DATABASE_PATH + '/', '').split('/')
        person_name = identity_path[0]
        
        # Store first occurrence of each person
        if person_name not in unique_individuals:
            unique_individuals[person_name] = prediction
    return unique_individuals

def recognize_face(test_image_path):
    """
    Recognizes faces in an image using DeepFace
    
    Args:
        test_image_path: Path or array of image for recognition
    
    Returns:
        dict: Dictionary of unique individuals with their info
    """
    try:
        # Perform face recognition
        dfs = DeepFace.find(
            img_path=test_image_path,
            db_path=DATABASE_PATH,
            model_name=MODEL,
            detector_backend=BACKEND,
            distance_metric=DISTANCE_METRIC,
            enforce_detection=False,
            threshold=THRESHOLD)
        
        # Process the results
        top_predictions = get_top_predictions(dfs)
        unique_individuals = process_predictions(top_predictions)
        return unique_individuals
    except Exception as e:
        print(f"\nAn error occurred recognizing {test_image_path} with model {MODEL} and backend {BACKEND}: {e}\n")
        return None

def fetch_image_with_timeout(url, queue, timeout=STREAM_TIMEOUT):
    """
    Fetches an image from a camera URL with timeout protection
    
    Args:
        url: Camera stream URL
        queue: Queue to store retrieved image
        timeout: Maximum wait time in seconds
    """
    try:
        resp = urllib.request.urlopen(url, timeout=timeout)
        img_np = np.array(bytearray(resp.read()), dtype=np.uint8)
        img = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
        queue.append(img)
    except Exception as e:
        print(f"Timeout or error fetching image from {url}: {e}")
        queue.append(None)

def get_object_bounding_boxes(image, label_filter=None):
    """
    Detects objects in image and returns their bounding boxes and labels
    
    Args:
        image: Input image (BGR format)
        label_filter: List of object labels to detect (None for all objects)
    
    Returns:
        tuple: (list of bounding boxes, list of corresponding labels)
    """
    results = object_detection_model(image, device="mps")
    result = results[0]
    bboxes = np.array(result.boxes.xyxy.cpu(), dtype="int")
    class_ids = np.array(result.boxes.cls.cpu(), dtype="int")
    
    filtered_bboxes = []
    filtered_labels = []
    
    for cls, bbox in zip(class_ids, bboxes):
        if label_filter is None or classes[cls] in label_filter:
            (x, y, x2, y2) = bbox
            filtered_bboxes.append((x, y, x2 - x, y2 - y))
            filtered_labels.append(classes[cls])
            
    return filtered_bboxes, filtered_labels

def calculate_average_depth(points_3D, bbox, allowed_depth=0.95):
    """
    Calculates average depth within a bounding box region
    
    Args:
        points_3D: 3D points map from stereo disparity
        bbox: Object bounding box (x, y, w, h)
        allowed_depth: Depth filter threshold (0.0-1.0)
    
    Returns:
        float: Average depth value or None if invalid
    """
    x, y, w, h = bbox
    obj_region = points_3D[y:y+h, x:x+w]
    
    # Get valid depth values
    valid_depths = obj_region[:, :, 2]
    valid_depths = valid_depths[np.isfinite(valid_depths)]
    
    if valid_depths.size == 0:
        return None
        
    # Filter outliers
    max_depth = np.max(valid_depths)
    filtered_depths = valid_depths[valid_depths <= allowed_depth * max_depth]
    
    if filtered_depths.size == 0:
        return None
        
    # Calculate average
    average_depth = np.mean(filtered_depths)
    return average_depth

def get_stereo_images(url_left, url_right):
    """
    Captures synchronized images from both cameras
    
    Args:
        url_left: Left camera URL
        url_right: Right camera URL
    
    Returns:
        tuple: (left_image, right_image)
    """
    queue_left, queue_right = [], []

    # Start parallel image capture threads
    thread_left = threading.Thread(target=fetch_image_with_timeout, args=(url_left, queue_left))
    thread_right = threading.Thread(target=fetch_image_with_timeout, args=(url_right, queue_right))
    
    thread_left.start()
    thread_right.start()
    thread_left.join()
    thread_right.join()
    
    img_left = queue_left[0]
    img_right = queue_right[0]
    
    return img_left, img_right

def preprocess_frames():
    """
    Preprocesses stereo camera input, returning:
    - rectified image used for display
    - recognized face names (if any)
    - object depth info (if stereo and available)
    """
    # Track stream health
    img_left, img_right = get_stereo_images(esp32_left_image_url, esp32_right_image_url)

    if img_left is None and img_right is None:
        print("Both images are None.")
        return None, None, None

    img_left_rectified = rectify_left_image(img_left) if img_left is not None else None
    img_right_rectified = rectify_right_image(img_right) if img_right is not None else None

    if img_left_rectified is not None:
        faces = recognize_face(img_left_rectified)
        object_bboxes, object_labels = get_object_bounding_boxes(img_left_rectified, label_filter=LABELS)
        img_rectified = img_left_rectified
    elif img_right_rectified is not None:
        faces = recognize_face(img_right_rectified)
        object_bboxes, object_labels = get_object_bounding_boxes(img_right_rectified, label_filter=LABELS)
        img_rectified = img_right_rectified
    else:
        return None, None, None

    object_depths = None

    if img_left_rectified is not None and img_right_rectified is not None:
        if faces or (object_bboxes and len(object_bboxes) > 0):
            disp_norm, points_3D = calculate_disparity_maps(stereo, img_left_rectified, img_right_rectified)

            if faces:
                draw_boxes_and_labels(img_left_rectified, faces)

            object_depths = []
            for bbox, label in zip(object_bboxes, object_labels):
                x, y, w, h = bbox
                cv2.rectangle(disp_norm, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.putText(disp_norm, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                cv2.rectangle(img_left_rectified, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(img_left_rectified, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                depth = calculate_average_depth(points_3D, bbox, allowed_depth=ALLOWED_DEPTH)
                object_depths.append((label, depth))
                if depth is not None:
                    cv2.putText(disp_norm, f"Depth: {depth:.2f}", (x, y + h + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                    print(f"{label} detected. Average {label} depth: {depth:.2f}")

            combined = np.concatenate((cv2.cvtColor(disp_norm, cv2.COLOR_GRAY2BGR), img_left_rectified), axis=1)
            cv2.imshow("Disparity map", combined)
        else:
            cv2.imshow("Disparity map", img_left_rectified)

    else:
        if faces:
            draw_boxes_and_labels(img_rectified, faces)
        for bbox, label in zip(object_bboxes, object_labels):
            x, y, w, h = bbox
            cv2.rectangle(img_rectified, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img_rectified, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow("Disparity map", img_rectified)

    return img_rectified, list(faces.keys()) if faces else None, object_depths

def main():
    """
    Main loop to call preprocess_frames() and handle display loop
    """
    update_camera_config(esp32_left_config_url, JPEG_QUALITY, FRAME_SIZE)
    update_camera_config(esp32_right_config_url, JPEG_QUALITY, FRAME_SIZE)
    DeepFace.build_model(MODEL)

    while True:
        _, _, _ = preprocess_frames()  # Process and display
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()