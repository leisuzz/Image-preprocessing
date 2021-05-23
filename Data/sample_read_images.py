#Sample Python script for Assignment 1

import skimage.io
import matplotlib.pyplot as plt
from skimage.color import rgb2gray

# Read image data
us_img = skimage.io.imread('dataset1_img_hip.png')
mask = skimage.io.imread('dataset1_mask_hip.png')

# Convert image to grayscale
us_img = rgb2gray(us_img)
mask   = rgb2gray(mask)

fig, ax = plt.subplots(nrows=1, ncols=2)
ax[0].imshow(us_img, cmap='gray')
ax[1].imshow(mask, cmap='gray')
plt.show()

# Apply Enhancement



# Calculate indices
    # Convert to double if necessary
    # Multiply by mask
