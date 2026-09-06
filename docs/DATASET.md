# Dataset

**Source:** India Driving Dataset (IDD)
**Format:** PASCAL VOC / COCO JSON (Detection) & leftImg8bit / gtFine (Segmentation)

### Subsetting Strategy
Due to the immense size of the full IDD dataset (~50GB), we implemented an automated subset extraction script (`scripts/prepare_data.py`). 
- It inspects the `.tar.gz` archives without full extraction.
- It pulls a matched subset of images and annotations directly into memory and writes them to `data/subset/`.
- This ensures we don't duplicate data and can run quick iterations on a MacBook.
