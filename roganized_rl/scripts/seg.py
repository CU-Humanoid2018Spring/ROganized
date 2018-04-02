import cv2
import numpy as np
import matplotlib.pyplot as plt

# TODO: define HSV ranges here
beer = {'name':'beer', 'lo':np.array([40,40,40]), 'hi':np.array([80,80,80]), 'val': [255,0,0]}
demo_cube = {'name':'demo_cube', 'lo': np.array([100,100,100]), 'hi':np.array([200,200,200]), 'val': [0,255,0]}
img = cv2.imread('frame-000007.segm.png')

# Actual process pipeline
output = np.zeros(img.shape, np.uint8)
for target in [beer,demo_cube]:
    print target['name']
    mask = cv2.inRange(img,target['lo'],target['hi'])
    rows,cols =  mask.nonzero()
    for i in range(rows.shape[0]):
        output[rows[i]][cols[i]] = target['val']
plt.imshow(output)
plt.show()

# Save output
#filename = 'SEGMENTED.png'
#cv2.imwrite(filename, output)
