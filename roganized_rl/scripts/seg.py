#!/usr/bin/python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import glob
import os
from natsort import natsorted

beer = {'name':'beer', 'lo':np.array([0,100,80]), 'hi':np.array([8,200,200]), 'val': 1}
cricket_ball = {'name': 'cricket_ball', 'lo': np.array([0,200,100]), 'hi': np.array([30,255,255]), 'val': 2}
demo_cube = {'name':'demo_cube', 'lo': np.array([110,50,50]),'hi':np.array([130,255,255]), 'val': 3}
hammer = {'name':'hammer', 'lo': np.array([60,60,100]), 'hi': np.array([80,256,150]), 'val': 4}
wood_cube = {'name':'wood_cube', 'lo':np.array([9,80,80]), 'hi':np.array([16,200,130]), 'val': 5}


def segment_image(filename):
	'''
	The segmentation pipeline, read an image and save the segemented image.
	'''
	img = cv2.imread(filename)
	if img is None:
		raise Exception('invalid fileaname')
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	output = np.zeros((hsv.shape[0],hsv.shape[1],1), np.uint8)
	kernel = np.ones((5,5),np.uint8)
	for target in [beer, cricket_ball,demo_cube,hammer,wood_cube]:
		mask = cv2.inRange(hsv,target['lo'],target['hi'])
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		rows,cols =  mask.nonzero()
		for i in range(rows.shape[0]):
			output[rows[i]][cols[i]] = target['val']
	return output

def glob_png_files(path):
	'''
	Retrieve file batches in the following structure:
	 - batch[0]
	    - batch: 0
	    - images: [scene0.png, scene1.png, ..., scene99.png]
	 - batch[1]
	    - batch: 1
	    - images: [scene100.png, scene101.png, ..., scene199.png]
	'''
	sources = os.listdir(path)
	sources = natsorted(sources,key=lambda y : y.split('_')[-1])
	batches = []
	for source in sources:
		batch = {'batch': source, 'images': glob.glob(path+'/'+source+'/*.png')}
		batches.append(batch)
	return batches


def main():
	parser = argparse.ArgumentParser(description='Project ROganzied - Image segmentation pipeline')
	parser.add_argument('-s', '--src', help='absolute source path', required=True)
	parser.add_argument('-d', '--dest', help='absolute path to save images', required=True)
	
	# Create output directory
	dir_out = parser.parse_args().save
	try:
		os.stat(dir_out)
	except:
		print 'mkdir ', dir_out
		os.mkdir(dir_out)

	# Glob file lists
	filepath = parser.parse_args().path
	batches  = glob_png_files(filepath)

	for batch in batches: 
		print batch['batch']
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

if __name__ == '__main__':
	main()