#!/usr/bin/env python

# Run: $ rospy roganized_gazebo generate_data.py <img_dir> <count>
# ex.: $ rospy roganized_gazebo generate_data.py four_cubes 30000

from roganized_gazebo.table_manager import TableManager
from roganized_rl.utils import ImageConverter
import cv2
import numpy as np
import rospy
import os
import sys

DATA_PATH = 'data'
BATCH_START = 0
REFS = ["blank-table.png", "blank-table-2.png", "blank-table-3.png"]
BATCH_SIZE = 100


def setup_base_dir(img_dir, data_path=DATA_PATH, batch_num=BATCH_START):
    """
        Setup directory within /data/batch_0 for saving images in src/ROganized. 
        Creates /data if necessary.
        Returns img_dir
    """
    cwd = os.getcwd()
    if os.path.basename(cwd) == 'ROganized':
        data_path = os.path.join(os.getcwd(), data_path)
    elif os.path.basename(cwd) == 'team2_ws':
        data_path = os.path.join(os.getcwd(), "src/ROganized", data_path)
    elif os.path.basename(cwd) == 'src':
        data_path = os.path.join(os.getcwd(), "ROganized", data_path)
    else:
        data_path = data_path

    if not os.path.exists(os.path.join(data_path, img_dir)):
        os.makedirs(os.path.join(data_path, img_dir))
        print("Making path to ", os.path.join(data_path, img_dir))
    
    return data_path, batch_num
    
    
def update_cur_dir(batch_num, img_dir):
    """Create new directory for next batch of images. Returns updated batch_num, updated cur_dir."""
    cur_dir = os.path.join(data_path, img_dir, "batch_" + str(batch_num))
    if not os.path.exists(cur_dir):
        os.makedirs(cur_dir)
        print("Making batch directory: ", cur_dir)
    return cur_dir
    

def same_img(img, ref):
    """Check if cv2 images are identical."""
    identical = (img.shape == ref.shape) and not (np.bitwise_xor(img, ref).any())
    # if identical:
    #     print("same image detected")
    return identical
    

if __name__ == '__main__':
    img_dir = sys.argv[1]
    count = 100 if len(sys.argv) < 2 else int(sys.argv[2])
    print("Generating %i images" % count)
    
    img_src = ImageConverter()
    table = TableManager()
    table.clear()
    #grid = np.array([[1,0,0,0],
    #                 [0,0,1,0],
    #                 [1,0,0,0],
    #                 [0,0,0,1]])
    table.spawn()
    
    data_path, batch_num = setup_base_dir(img_dir)
    cur_dir = update_cur_dir(batch_num, img_dir)
    ref_imgs = []
    for img_name in REFS:
        ref_path = os.path.join(data_path, img_name)
        print(ref_path)
        ref_imgs.append(cv2.imread(ref_path))
    print(ref_imgs)

    img_count = 0
    while not rospy.is_shutdown() and img_count < count:
        n = len(os.listdir(cur_dir))
        
        # Generate scene
        positions = np.random.choice(15,4,replace=False)
        for i, position in enumerate(positions):
            table.move_cube(i, position/4, position%4)
            
        # Stabilize
        rospy.sleep(0.2)
        
        # Create a new batch image directory if needed
        if img_count > 1 and n % BATCH_SIZE == 0:
            batch_num += 1
            cur_dir = update_cur_dir(batch_num, img_dir)
            
        # Get img
        img = img_src.get_rgb()
        if img is None:
            continue
        elif sum([same_img(img, ref) for ref in ref_imgs]) == 0:
            cv2.imwrite(img_path, img)
            img_count += 1

    print("Generated %i images in %s" % (img_count, data_path))
