#!/usr/bin/python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import glob
import os

# TODO: define HSV ranges here
beer = {'name':'beer', 'lo':np.array([0,100,80]), 'hi':np.array([8,200,200]), 'val': [255,255,0]}
cricket_ball = {'name': 'cricket_ball', 'lo': np.array([0,200,100]), 'hi': np.array([30,255,255]), 'val': [0,0,255]}
demo_cube = {'name':'demo_cube', 'lo': np.array([110,50,50]),'hi':np.array([130,255,255]), 'val': [0,255,0]}
hammer = {'name':'hammer', 'lo': np.array([60,60,100]), 'hi': np.array([80,256,150]), 'val': [0,255,255]}
wood_cube = {'name':'wood_cube', 'lo':np.array([9,80,80]), 'hi':np.array([16,200,130]), 'val': [255,255,255]}


def segment_image(filename):
	'''
	The segmentation pipeline, read an image and save the segemented image.
	'''
	img = cv2.imread(filename)
	if img is None:
		raise Exception('invalid fileaname')
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	output = np.zeros(hsv.shape, np.uint8)
	kernel = np.ones((5,5),np.uint8)
	for target in [beer, cricket_ball,demo_cube,hammer,wood_cube]:
		mask = cv2.inRange(hsv,target['lo'],target['hi'])
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		rows,cols =  mask.nonzero()
		for i in range(rows.shape[0]):
			output[rows[i]][cols[i]] = target['val']
	return output

def glob_png_files(path):
	sources = os.listdir(path)
	batches = []
	for source in sources:
		batch = {'batch': source, 'images': glob.glob(path+'/'+source+'/*.png')}
		batches.append(batch)
	return batches

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Image path')
	parser.add_argument('-p', '--path', help='relative path', required=True)
	parser.add_argument('-s', '--save', help='directory to save images')
	
	# Create output directory if specified
	dir_out = parser.parse_args().save
	if dir_out is not None:
		try:
			os.stat(dir_out)
			#raise Exception('Output directory exists. Please specify other path.')
		except:
			print mkdir, dir_out
			os.mkdir(dir_out)

	# Glob file lists
	filepath = parser.parse_args().path
	batches  = glob_png_files(filepath)

	for batch in batches[:1]:
		dest = dir_out+'/'+batch['batch']
		try:
			os.stat(dest)
		except:
			print 'mkdir ',dest
			os.mkdir(dest)
		for filein in batch['images']:
			segmented = segment_image(filein)
			fileout = dest+'/'+filein.split('/')[-1]
			cv2.imwrite(fileout, segmented)
			
	#segment_image(filename)
