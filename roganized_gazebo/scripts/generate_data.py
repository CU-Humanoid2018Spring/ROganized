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
prefix = 'scene'
suffix = '.png'


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
    """Create new directory for next batch of images. Returns updated cur_dir path."""
    cur_dir = os.path.join(data_path, img_dir, "batch_" + str(batch_num))
    if not os.path.exists(cur_dir):
        os.makedirs(cur_dir)
        print("Making batch directory: ", cur_dir)
    return cur_dir
    

def same_img(img, ref):
    """Check if cv2 images are identical to filter out blank tables."""
    identical = (img.shape == ref.shape) and not (np.bitwise_xor(img, ref).any())
    # if identical:
    #     print("same image detected")
    return identical
    

if __name__ == '__main__':
    # Capture input args <img_dir> and <num_images>
    img_dir = sys.argv[1]
    count = 100 if len(sys.argv) < 2 else int(sys.argv[2])
    
    # Setup working paths
    data_path, batch_num = setup_base_dir(img_dir)
    cur_dir = update_cur_dir(batch_num, img_dir)
    
    print("Generating %i images in directory %s" % (count, os.join(data_path, img_dir)))
    
    # Setup blank table images to filter out
    ref_imgs = []
    for img_name in REFS:
        ref_path = os.path.join(data_path, img_name)
        print(ref_path)
        ref_imgs.append(cv2.imread(ref_path))
    failed_imgs = [i for i, img in enumerate(ref_imgs) if img is None]
    if len(failed_imgs) > 0:
        print("The following reference images failed to load:", REFS[failed_imgs])
    
    # Initialize ROS managers
    img_src = ImageConverter()
    table = TableManager()
    table.clear()
    table.spawn()

    img_count = 0
    scores = []
    while not rospy.is_shutdown() and img_count < count:
        n = len(os.listdir(cur_dir))
        
        # Generate scene
        positions = np.random.choice(15,4,replace=False)
        for i, position in enumerate(positions):
            table.move_cube(i, position/4, position%4)
            
        # Stabilize
        rospy.sleep(0.2)
        
        # Create a new batch image directory if needed, saving .npy file first.
        if img_count > 1 and n % BATCH_SIZE == 0:
            scores_path = os.join(cur_dir, "batch_%i_scores" % batch_num)
            np.save(scores_path, scores)
            scores = []
            batch_num += 1
            cur_dir = update_cur_dir(batch_num, img_dir)
            
        # Get img and score
        img = img_src.get_rgb()
        score = table.score()
        if img is None:
            continue
        elif sum([same_img(img, ref) for ref in ref_imgs]) == 0:
            img_name = prefix + str(img_count) + suffix
            img_path = os.path.join(cur_dir, img_name)
            cv2.imwrite(img_path, img)
            scores.append(score)
            img_count += 1

    print("Generated %i images in %s" % (img_count, data_path))
