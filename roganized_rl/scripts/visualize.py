#!/usr/bin/python
import cv2
import numpy as np
import argparse

def main():

	parser = argparse.ArgumentParser(description='Project ROganzied - Visualize segmented images')
	parser.add_argument('-p', '--path', help='absolute source path', required=True)
	filepath = parser.parse_args().path

	img = cv2.imread(filepath)
	if img is None:
		raise Exception(filepath + ' does not exist')

	# Define customize lookup table
	lut = np.zeros((256,1,3),dtype=np.uint8)
	lut[1,:,:] = [255,0,0]
	lut[2,:,:] = [0,255,0]
	lut[3,:,:] = [0,0,255]
	lut[4,:,:] = [0,255,255]
	lut[5,:,:] = [255,255,0]
	print '--------.---------'
	print 'color\t|object'
	print '--------|---------'
	print 'blue\t|beer'
	print 'green\t|cricket ball'
	print 'red\t|demo cube'
	print 'yellow\t|hammer'
	print 'cyan\t|wood cube'

	rgb = cv2.LUT(img,lut)
	cv2.imshow('class visualizer', rgb)
	cv2.waitKey(0)

if __name__ == '__main__':
	main()