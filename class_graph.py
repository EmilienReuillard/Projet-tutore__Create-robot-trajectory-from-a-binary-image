import numpy as np
from PIL import Image
import skimage.io as io 
import skimage
import skimage.filters
from skimage import measure,filters,morphology
from skimage.measure import label,regionprops
from skimage.transform import hough_line, hough_line_peaks, warp, AffineTransform
from skimage.feature import canny, corner_harris, corner_subpix, corner_peaks
from skimage.draw import line
from skimage import data
import matplotlib.pyplot as plt
from matplotlib import cm
import scipy
from math import*

class graph:
    
#_______VARIABLES_______   
    coords_peaks = []
    lst_connections = []
    N_peaks = 0
    
#______FONCTIONS_____

    def __init__(self,path_image):
        #ouverture de l'image
        self.image = io.imread(path_image)
        self.image_edges = canny(self.image,3)
        self.image_blurred = skimage.filters.gaussian(self.image_edges,(1,1))
        
        #Détection des coins dans l'image
        self.coords_peaks = corner_peaks(corner_harris(self.image), min_distance=5, threshold_rel=0.02)
        self.N_peaks = len(self.coords_peaks)
        
    #Fonction Vérifie la connection entre 2 points via des lignes droites
    def connect2points(self,pt1,pt2):
        x1 = self.coords_peaks[pt1][0]
        y1 = self.coords_peaks[pt1][1]
        x2 = self.coords_peaks[pt2][0]
        y2 = self.coords_peaks[pt2][1]
        
        l = sqrt((y2-y1)**2 + (x2-x1)**2)   #distance entre les 2 points
        L = int(l)  #distance entière
        lst_pt_x = np.linspace(x1,x2,L) #liste qui regroupes les coordonées en x à chaque pas de temps
        lst_pt_y = np.linspace(y1,y2,L) #liste qui regroupes les coordonées en y à chaque pas de temps
        
        #print(f"x1={x1} y1={y1} x2={x2} y2={y2}")
        #print(f"L = {L}")
        
        compt = 0
        i,j = int(lst_pt_x[0]),int(lst_pt_y[0])
        while( self.image_blurred[i][j] != 0 ):
            compt +=1
            i = int(lst_pt_x[compt])
            j = int(lst_pt_y[compt])
            
            if (( i >= x2-5 and i <= x2+5 ) and (j >= y2-5 and j <= y2+5)) :
                #print(f"{pt1} & {pt2} connected\n")
                return True
            
            if(self.image_blurred[i][j] == 0):
                #print(f"no connection beetween pt1:{pt1} & pt2:{pt2}")
                return False
            
    #fonction qui liste les connexions de chaque point
    def mapping_connexions(self):
        for i in range(self.N_peaks):
            for j in range(self.N_peaks):
                if i != j:
                    if self.connect2points(i,j) == True:
                        self.lst_connections.append([])
                        self.lst_connections[i].append(i)
                        self.lst_connections[i].append(j)

