import cv2
import numpy as np
from ultralytics import YOLO

class Detector:
    def __init__(self, model_path='yolov8n.pt', conf_threshold=0.3):
        print(f"Loading YOLO model from {model_path}...")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        
        # Mapping COCO classes to our categories where possible
        self.class_map = {
            0: 'pedestrian',
            1: 'bicycle',
            2: 'car',
            3: 'motorcycle',
            5: 'bus',
            7: 'truck',
            9: 'traffic light',
            10: 'traffic sign',
            15: 'animal', # cat
            16: 'animal', # dog
            17: 'animal', # horse
            18: 'animal', # sheep
            19: 'animal', # cow
            20: 'animal', # elephant
            21: 'animal', # bear
            22: 'animal'  # zebra
        }

    def detect(self, image):
        """
        Runs detection on an image.
        Returns a list of dicts:
        {
            'class': str,
            'confidence': float,
            'bbox': [x1, y1, x2, y2],
            'center': [cx, cy]
        }
        """
        results = self.model(image, verbose=False)[0]
        detections = []
        
        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < self.conf_threshold:
                continue
                
            cls_id = int(box.cls[0])
            
            # Use our mapped class or default to coco name
            if cls_id in self.class_map:
                class_name = self.class_map[cls_id]
            else:
                # Optionally ignore other classes or map them as generic obstacle
                class_name = self.model.names[cls_id]
                
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            
            detections.append({
                'class': class_name,
                'confidence': conf,
                'bbox': [x1, y1, x2, y2],
                'center': [cx, cy],
                'estimated_position': None # To be filled by tracking/depth
            })
            
        return detections
        
    def visualize(self, image, detections, track_ids=None):
        """
        Draws bounding boxes and labels on the image.
        """
        vis_img = image.copy()
        for i, det in enumerate(detections):
            x1, y1, x2, y2 = map(int, det['bbox'])
            label = f"{det['class']} {det['confidence']:.2f}"
            
            if track_ids and i < len(track_ids) and track_ids[i] is not None:
                label = f"ID:{track_ids[i]} " + label
                
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis_img, label, (x1, max(y1 - 10, 0)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
        return vis_img

if __name__ == "__main__":
    # Test the detector
    detector = Detector()
    print("Detector initialized successfully.")
