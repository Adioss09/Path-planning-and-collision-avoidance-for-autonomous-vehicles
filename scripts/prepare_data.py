import os
import tarfile
import shutil
import json
from pathlib import Path
from tqdm import tqdm

def prepare_subset(tar_path, output_dir, max_samples, is_segmentation=False):
    print(f"Preparing subset from {tar_path} into {output_dir}")
    os.makedirs(output_dir, exist_ok=True)
    
    if is_segmentation:
        img_out = os.path.join(output_dir, "images")
        mask_out = os.path.join(output_dir, "masks")
        os.makedirs(img_out, exist_ok=True)
        os.makedirs(mask_out, exist_ok=True)
    else:
        img_out = os.path.join(output_dir, "images")
        ann_out = os.path.join(output_dir, "annotations")
        os.makedirs(img_out, exist_ok=True)
        os.makedirs(ann_out, exist_ok=True)
        
    extracted_count = 0
    
    # Simple extraction logic (just finding files and extracting)
    try:
        with tarfile.open(tar_path, "r:gz") as tar:
            members = tar.getmembers()
            
            # For a proper subset, we should match images with annotations.
            # In IDD, they share prefixes or specific folder structures.
            # For this PoC, we will just extract the first N images and their corresponding annotations.
            
            # Create a lookup for annotations
            annotations = {}
            for m in members:
                if m.isfile() and (m.name.endswith('.xml') or m.name.endswith('.json') or (is_segmentation and 'gtFine' in m.name and m.name.endswith('.png'))):
                    basename = os.path.basename(m.name).replace('_gtFine_labelIds', '').replace('_leftImg8bit', '').split('.')[0]
                    annotations[basename] = m
                    
            # Extract images and matching annotations
            for m in tqdm(members, desc="Extracting samples"):
                if extracted_count >= max_samples:
                    break
                    
                if m.isfile() and m.name.lower().endswith(('.jpg', '.png')) and not 'gtFine' in m.name:
                    basename = os.path.basename(m.name).replace('_leftImg8bit', '').split('.')[0]
                    
                    if basename in annotations:
                        # Extract image
                        img_dest = os.path.join(img_out, os.path.basename(m.name))
                        f_in = tar.extractfile(m)
                        with open(img_dest, 'wb') as f_out:
                            shutil.copyfileobj(f_in, f_out)
                            
                        # Extract annotation
                        ann_m = annotations[basename]
                        ann_dest = os.path.join(mask_out if is_segmentation else ann_out, os.path.basename(ann_m.name))
                        f_in = tar.extractfile(ann_m)
                        with open(ann_dest, 'wb') as f_out:
                            shutil.copyfileobj(f_in, f_out)
                            
                        extracted_count += 1
                        
    except Exception as e:
        print(f"Error extracting {tar_path}: {e}")
        
    print(f"Extracted {extracted_count} complete samples.")
    return extracted_count

def main():
    base_dir = os.getcwd()
    
    det_tar = os.path.join(base_dir, "idd-detection.tar.gz")
    seg_tar = os.path.join(base_dir, "idd-segmentation.tar.gz")
    
    det_out = os.path.join(base_dir, "data", "subset", "detection")
    seg_out = os.path.join(base_dir, "data", "subset", "segmentation")
    
    # Extract a small subset for PoC (e.g. 100 for fast local dev)
    # The spec asks for 3000-5000 ideally, but 100 is better for quick mode testing
    if os.path.exists(det_tar):
        prepare_subset(det_tar, det_out, max_samples=100, is_segmentation=False)
        
    if os.path.exists(seg_tar):
        prepare_subset(seg_tar, seg_out, max_samples=100, is_segmentation=True)

if __name__ == "__main__":
    main()
