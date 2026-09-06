import cv2
import numpy as np
import torch
import torchvision.transforms as T
from torchvision.models.segmentation import lraspp_mobilenet_v3_large, LRASPP_MobileNet_V3_Large_Weights

class Segmenter:
    def __init__(self, use_ml=True):
        self.use_ml = use_ml
        self.device = torch.device('mps' if torch.backends.mps.is_available() else 'cpu')
        
        if self.use_ml:
            print("Loading lightweight segmentation model (LRASPP MobileNetV3)...")
            weights = LRASPP_MobileNet_V3_Large_Weights.DEFAULT
            self.model = lraspp_mobilenet_v3_large(weights=weights).to(self.device)
            self.model.eval()
            self.preprocess = weights.transforms()
            # COCO/Pascal VOC class 0 is usually background, we can assume road is somewhere in the scene.
            # But torchvision pre-trained models on COCO don't have a specific "road" class.
            # Actually, lraspp_mobilenet is trained on COCO subset (Pascal VOC 2012)
            # which doesn't have road. For a robust PoC without training on IDD, 
            # we can fallback to heuristic road segmentation or just simulate it for non-ML.
            
    def segment(self, image, gt_mask_path=None):
        """
        Segments the road/drivable area.
        Returns a binary mask where 255 is drivable area, 0 is obstacle/background.
        """
        if gt_mask_path is not None:
            # Use provided IDD ground truth mask for perfectly accurate simulation
            mask = cv2.imread(gt_mask_path, cv2.IMREAD_GRAYSCALE)
            if mask is not None:
                # IDD road class is typically 0 or a specific id, but we binarize appropriately
                # For simplicity, if we provide a gt mask, assume we already processed it.
                # Just return it thresholded
                _, bin_mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
                return bin_mask
                
        # Simulated heuristic segmenter for PoC (finds gray road-like areas)
        # This is a fallback if no ML model or GT is present
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for gray/asphalt color
        lower_gray = np.array([0, 0, 50])
        upper_gray = np.array([180, 50, 200])
        
        mask = cv2.inRange(hsv, lower_gray, upper_gray)
        
        # Morphological operations to clean up
        kernel = np.ones((15, 15), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Assume the bottom center of the image is always road (ego vehicle perspective)
        h, w = mask.shape
        cv2.circle(mask, (w//2, h - 50), 50, 255, -1)
        
        # Keep only the largest connected component connected to the bottom center
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num_labels > 1:
            bottom_center_label = labels[h - 50, w//2]
            if bottom_center_label > 0:
                mask = np.where(labels == bottom_center_label, 255, 0).astype(np.uint8)
                
        return mask

    def visualize(self, image, mask):
        """
        Overlays the mask on the image.
        """
        colored_mask = np.zeros_like(image)
        colored_mask[mask == 255] = [0, 255, 0] # Green drivable area
        
        vis_img = cv2.addWeighted(image, 0.7, colored_mask, 0.3, 0)
        return vis_img

if __name__ == "__main__":
    segmenter = Segmenter(use_ml=False)
    print("Segmenter initialized successfully.")
