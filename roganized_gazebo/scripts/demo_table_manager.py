#!/usr/bin/env python

from roganized_gazebo.table_manager import TableManager
import numpy as np
import rospy
if __name__ == '__main__':
    table = TableManager()
    table.clear()
    #grid = np.array([[1,0,0,0],
    #                 [0,0,1,0],
    #                 [1,0,0,0],
    #                 [0,0,0,1]])
    table.spawn()
    while not rospy.is_shutdown():
        positions = np.random.choice(15,4,replace=False)
        for i, position in enumerate(positions):
            table.move_cube(i, position/4, position%4)
        rospy.sleep(0.2)
