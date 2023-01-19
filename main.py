import numpy as np
from PIL import Image
import skimage.io as io 
import skimage
from skimage import measure,filters,morphology
from skimage.measure import label,regionprops
from skimage.transform import hough_line, hough_line_peaks, warp, AffineTransform
from skimage.feature import canny, corner_harris, corner_subpix, corner_peaks
from skimage.draw import line
from skimage import data
import matplotlib.pyplot as plt
from matplotlib import cm
import scipy
import math

#%%
#ouverture de l'image
Image = io.imread("Pieces.png")
A = io.imread("A.png")
plt.imshow(A,cmap="gray")
plt.show()

#%%
#Détection du nombre d'objets
"""
N_objet = 0
seuil = 100

Label_image = skimage.morphology.label(A)
for region in skimage.measure.regionprops(Label_image):
    N_objet += 1
print("N_objets = ",N_objet)
"""

#%%
#détection de coins
A_Harris = skimage.feature.corner_harris(A)
plt.figure(0)
plt.imshow(A_Harris,cmap="gray")
plt.show()

#on va travailler avec A_harris plus tard
A_Harris_abs = abs(A_Harris)
plt.figure(1)
plt.imshow(A_Harris_abs, cmap = "gray")
plt.show()

#def de fonction
def recup_coins(Image,seuil):
    H = len(Image)
    L = len(Image[0])
    lst_result = np.zeros((H,L))
    for i in range(H):
        for j in range(L):
            if Image[i][j] > seuil:
                lst_result[i][j] = 1
                
    return lst_result

lst_corner = recup_coins(A_Harris,5)
plt.figure(2)
plt.imshow(lst_corner, cmap = "gray")

#%%
#détection de coins2

#Négatif d'une image
A = np.invert(A)

coords = corner_peaks(corner_harris(A), min_distance=5, threshold_rel=0.02)
coords_subpix = corner_subpix(A, coords, window_size=13)

fig, ax = plt.subplots()
ax.imshow(A, cmap=plt.cm.gray)
ax.plot(coords[:, 1], coords[:, 0], color='cyan', marker='o',
        linestyle='None', markersize=6)
ax.plot(coords_subpix[:, 1], coords_subpix[:, 0], '+r', markersize=15)
ax.axis((0, 770, 770, 0))
plt.show()

#%%
#test detect ligne

#bool A

bool_A = np.copy(A_Harris_abs)
for i in range(len(bool_A)):
    for j in range(len(bool_A[0])):
        if(bool_A[i][j] > 2 ):
            bool_A[i][j] = 1
        else:
            bool_A[i][j] = 0

# Classic straight-line Hough transform
# Set a precision of 0.5 degree.
tested_angles = np.linspace(-np.pi / 2, np.pi / 2, 360, endpoint=False)
h, theta, d = hough_line(bool_A, theta=tested_angles)

# Generating figure 1
fig, axes = plt.subplots(1, 3, figsize=(15, 6))
ax = axes.ravel()

ax[0].imshow(bool_A, cmap=cm.gray)
ax[0].set_title('Input image')
ax[0].set_axis_off()

angle_step = 0.5 * np.diff(theta).mean()
d_step = 0.5 * np.diff(d).mean()
bounds = [np.rad2deg(theta[0] - angle_step),
          np.rad2deg(theta[-1] + angle_step),
          d[-1] + d_step, d[0] - d_step]
ax[1].imshow(np.log(1 + h), extent=bounds, cmap=cm.gray, aspect=1 / 1.5)
ax[1].set_title('Hough transform')
ax[1].set_xlabel('Angles (degrees)')
ax[1].set_ylabel('Distance (pixels)')
ax[1].axis('image')

ax[2].imshow(bool_A, cmap=cm.gray)
ax[2].set_ylim((bool_A.shape[0], 0))
ax[2].set_axis_off()
ax[2].set_title('Detected lines')

for _, angle, dist in zip(*hough_line_peaks(h, theta, d)):
    (x0, y0) = dist * np.array([np.cos(angle), np.sin(angle)])
    ax[2].axline((x0, y0), slope=np.tan(angle + np.pi/2))

plt.tight_layout()
plt.show()
"""
#recup coordonnées coins
A_lst_corner = skimage.feature.corner_peaks(A,15)
print(A_lst_corner)
"""

