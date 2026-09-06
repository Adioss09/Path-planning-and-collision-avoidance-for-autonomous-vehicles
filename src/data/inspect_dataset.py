import os
import tarfile
import json
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

def inspect_tar(tar_path):
    print(f"Inspecting {tar_path}...")
    report = {
        "filename": os.path.basename(tar_path),
        "total_files": 0,
        "images": 0,
        "annotations": 0,
        "sample_structure": [],
        "classes_found": set()
    }
    
    try:
        with tarfile.open(tar_path, "r:gz") as tar:
            members = tar.getmembers()
            report["total_files"] = len(members)
            
            # Save a small sample of the structure (first 20 files/dirs)
            report["sample_structure"] = [m.name for m in members[:20]]
            
            sample_xml = None
            sample_json = None
            
            for m in members:
                if m.isfile():
                    if m.name.lower().endswith(('.jpg', '.png', '.jpeg')):
                        report["images"] += 1
                    elif m.name.lower().endswith('.xml'):
                        report["annotations"] += 1
                        if sample_xml is None:
                            sample_xml = m
                    elif m.name.lower().endswith('.json'):
                        report["annotations"] += 1
                        if sample_json is None:
                            sample_json = m
                            
            # Try to parse class names from a sample annotation
            if sample_xml:
                try:
                    f = tar.extractfile(sample_xml)
                    if f:
                        tree = ET.parse(f)
                        root = tree.getroot()
                        for obj in root.findall('object'):
                            name = obj.find('name')
                            if name is not None and name.text:
                                report["classes_found"].add(name.text)
                except Exception as e:
                    print(f"Error parsing sample XML: {e}")
                    
            if sample_json and not report["classes_found"]:
                try:
                    f = tar.extractfile(sample_json)
                    if f:
                        data = json.loads(f.read().decode('utf-8'))
                        # Try common JSON structures (like COCO or custom IDD)
                        if "categories" in data:
                            for cat in data["categories"]:
                                if "name" in cat:
                                    report["classes_found"].add(cat["name"])
                        elif "objects" in data:
                            for obj in data["objects"]:
                                if "label" in obj:
                                    report["classes_found"].add(obj["label"])
                except Exception as e:
                    print(f"Error parsing sample JSON: {e}")
                    
    except Exception as e:
        print(f"Failed to inspect {tar_path}: {e}")
        
    report["classes_found"] = list(report["classes_found"])
    return report

def main():
    base_dir = os.getcwd()
    results_dir = os.path.join(base_dir, "results")
    os.makedirs(results_dir, exist_ok=True)
    
    datasets = [f for f in os.listdir(base_dir) if f.endswith(".tar.gz") and "idd" in f.lower()]
    
    full_report = {}
    for d in datasets:
        report = inspect_tar(os.path.join(base_dir, d))
        full_report[d] = report
        
    report_path = os.path.join(results_dir, "dataset_report.json")
    with open(report_path, "w") as f:
        json.dump(full_report, f, indent=4)
        
    print(f"Dataset report saved to {report_path}")

if __name__ == "__main__":
    main()
