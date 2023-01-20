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


#%%
#ouverture de l'image
image = io.imread("A.png")
image_edges = canny(image, 3)
image_blurred = skimage.filters.gaussian(image_edges,(1,1))

#Détection des coins de l'image
coords = corner_peaks(corner_harris(image), min_distance=5, threshold_rel=0.02)
coords_subpix = corner_subpix(image, coords, window_size=13)

#%% 
#Création des graphes et de leurs interconnection

#Fonction Vérifie la connection entre 2 points via des lignes droites
def connect2points(pt1,pt2,image_blurred):
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]
    
    l = sqrt((y2-y1)**2 + (x2-x1)**2)   #distance entre les 2 points
    L = int(l)  #distance entière
    lst_pt_x = np.linspace(x1,x2,L) #liste qui regroupes les coordonées en x à chaque pas de temps
    lst_pt_y = np.linspace(y1,y2,L) #liste qui regroupes les coordonées en y à chaque pas de temps
    
    print(f"x1={x1} y1={y1} x2={x2} y2={y2}")
    print(f"L = {L}")
    
    compt = 0
    i,j = int(lst_pt_x[0]),int(lst_pt_y[0])
    while( image_blurred[i][j] != 0 ):
        compt +=1
        i = int(lst_pt_x[compt])
        j = int(lst_pt_y[compt])
        
        if (( i >= x2-5 and i <= x2+5 ) and (j >= y2-5 and j <= y2+5)) :
            print(f"pt1:{pt1} & pt2:{pt2} connected")
            return True
        
        if(image_blurred[i][j] == 0):
            print(f"no connection beetween pt1:{pt1} & pt2{pt2}")
            return False
        

#On test la fonction
connect2points(coords[0],coords[5],image_blurred) #la fonction fonctionne

#%%
#Fonction de Tri; Trie les points en N Ensembles joints de proche en proche
def Tri_coord(coords,image_blurred):
    N_corner = len(coords)  #Nombre de coins
    lst_pts = range(N_corner)    #[0,1,2,3,4,5,......,N_corner-1]
    lst_joints = []
    
    #On test toutes les connections possibles et on les mets dans lst_joints
    
    lock_first_pts = 0  #Vérouillage pour la première détection
    
    #test d'une première connexion evec les 2 premier points. On prendra le point 2 comme base pour la suite
    if(lock_first_pts == 0):
        for i in lst_pts:
            for j in lst_pts:
                if (i != j):
                    if( connect2points(coords[i],coords[j],image_blurred) == True):
                        lock_first_pts = 1
                        lst_joints.append(coords[i])
                        lst_joints.append(coords[j])
                        del lst_pts[i]
                        del lst_pts[j]
                        break
                        
    #Maintenant, on va faire un graphe logique de point à point
    
    compt = 1   #compteur qui commence à 1, car on commence à partir du second point
    stop_compt = 0
    while len(lst_pts) > 0:    
        for i in lst_pts:
            if( connect2points(lst_joints[compt],coords[i],image_blurred) == True):
                lst_joints.append(coords[i])
                del lst_pts[i]
                compt += 1
                break
                    
        stop_compt += 1
        if stop_compt>10000:
            print("Point seul, graphe incomplet")
            break
        
        
    

#%%
#Fonction qui renvoie une liste avec tous les éléments interconnertés
def InterConnectedPoints(coords, image_blurred):
    
    N_corner = len(coords)  #Nombre de coins
    N_ensembles = 0 #Nombre d'éléments joints
    lst_points = range(N_corner)    #[0,1,2,3,4,5,......,N_corner-1]
    lst_res = np.matrix([[],[],[],[]])
    
    compt = 0
    
    while len(lst_points)>0:
        
        #On va tester le premier coins de la liste avec tous les autres pour détecter une connexion entre 2 points
        for i in lst_points:
            if(connect2points(coords[i],coords[i],image_blurred) == True ):
                N_ensembles += 1
                np.append(lst_res[N_ensembles], coords[i])
                del lst_points[i]   #on retire i de la liste des points "seuls"
                break       
            
            
        compt += 1 
        if(compt > 10000): break
        
    

        
        
    
    
    


