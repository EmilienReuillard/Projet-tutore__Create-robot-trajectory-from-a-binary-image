
import numpy as np
from PIL import Image
import skimage.io as io 
import skimage
from skimage import measure,filters,morphology
from skimage.measure import label,regionprops
import matplotlib.pyplot as plt
import scipy

#%%
#ouverture de l'image
Image = io.imread("Pieces.png")
plt.imshow(Image,cmap="gray")
plt.show()


# %%
#Détection du nombre d'objets

N_objet = 0
seuil = 100

Label_image = skimage.morphology.label(Image)
for region in skimage.measure.regionprops(Label_image):
    N_objet += 1
print("N_objets = ",N_objet)
    
# %%
