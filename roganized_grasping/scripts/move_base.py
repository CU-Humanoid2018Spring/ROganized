#!/usr/bin/env python
from roganized_grasping.controller import move_base
from numpy import pi

if __name__ == '__main__':
    move_base(1,0, pi/3.0)
