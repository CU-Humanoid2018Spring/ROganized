#!/usr/bin/env python

from roganized_gazebo.table_manager import TableManager
from roganized_rl.utils import ImageConverter
import cv2
import numpy as np
import rospy

if __name__ == '__main__':
    img_src = ImageConverter()
    grids = 5
    table = TableManager(size=0.5, grids=grids)
    table.clear()
    table.spawn()
    while not rospy.is_shutdown():
        # Generate scene
        positions = np.random.choice(grids*grids-1,4,replace=False)
        for i, position in enumerate(positions):
            table.move_cube(i, position/grids, position%grids)

        print table.score()
        img = img_src.get_rgb()
        if img is None:
            continue
        cv2.imshow('shuffle', img)
        cv2.waitKey(3)
        rospy.sleep(1)
