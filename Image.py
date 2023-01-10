import numpy as np
from PIL import Image

class Image:
    Largeur = 0 #dimension en pixels
    Hauteur = 0 #dimension en pixels
    name = ""   #nom de l'image
    Image = np.array()
    
    def init_image(self,ref_image,name_image):
        self.name = name_image
        self.Image = Image.open(ref_image)
        self.Hauteur = len(self.Image)
        self.Largeur = len(self.Image[0])