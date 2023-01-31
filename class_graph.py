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

#test une liste pour savoir si elle contiens une variable
def appartient(lst,var):
    for i in range(len(lst)):
        if lst[i]==var:
            return True
    return False


class graph:
    
#_______VARIABLES_______   
    coords_peaks = []   #coordonées des coins [x,y]
    N_peaks = 0 #Nombre de coins dans l'image
    lst_connections = []    #Pour chaque pic, répértorie les points connectés au n-ieme de la liste
    lst_ensembles = []  #Répertorie les points interconnerctés
    N_ensembles = 0
    trajectory_pts = [[],[],[]]
    trajectory_pts_reel = [[],[],[]] #coordonnées réelles à parir d'un facteur d'échelle
    
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
            var = []
            for j in range(self.N_peaks):
                if i != j:
                    if self.connect2points(i,j) == True:
                        var.append(j)
            self.lst_connections.append(var)
                        
                        
    def ensembles(self):
        #On crée une liste qui liste tous les points qui n'ont pas encore été utilisés
        used_point = []
        first_point = 0 #premier point arbitraire
        used_point.append(first_point)
        
        last_point = 0
        next_point = self.lst_connections[0][0]
        var = []    #ensemble N que l'on ajoutera à lst_ensemble après
        var.append(first_point) #on ajoute first point à l'ensemble 1
        var.append(next_point)  #on ajoute le point 2 à l'ensemble 1
        
        while len(used_point) < len(self.lst_connections):
            
            #trouver le prochain point sans revenir en arrière
            for i in range(len(self.lst_connections[next_point])):
                if self.lst_connections[next_point][i] != last_point:
                    last_point = next_point
                    next_point = self.lst_connections[next_point][i]
                    
                    if next_point != first_point:
                        used_point.append(next_point)
                        var.append(next_point)      #on ajoute à l'ensemble n
                        
                    else:
                        self.lst_ensembles.append(var)
                        var = []

                        #recherche d'un élément inutilisé
                        for i in range(len(self.lst_connections)):
                            if i not in used_point:
                                first_point = i
                                last_point = first_point
                                next_point = self.lst_connections[first_point][0]
                                used_point.append(last_point)
                                used_point.append(next_point)
                                var.append(first_point)
                                var.append(next_point)
                                break
    
    #fait des points de trajectoire à partir des résultats trouvés par la fonction précédente
    def trajectory_points(self,pas=1):
        
        for i in range(len(self.lst_ensembles)):
            for j in range(len(self.lst_ensembles[i])):
                
                if j < len(self.lst_ensembles[i])-1:
                    n0 = self.lst_ensembles[i][j] #On récupère le n° du point que l'on veut utiliser
                    n1 = self.lst_ensembles[i][j+1]
                    #print(f"n0 = {n0}")
                    #print(f"n1 = {n1}")
                else:
                    n0 = self.lst_ensembles[i][j] #On récupère le n° du point que l'on veut utiliser
                    n1 = self.lst_ensembles[i][0]
                    
                x0 = self.coords_peaks[n0][0]
                y0 = self.coords_peaks[n0][1]
                
                x1 = self.coords_peaks[n1][0]
                y1 = self.coords_peaks[n1][1]
                
                L = sqrt((x1-x0)**2+(y1-y0)**2)
                
                for k in range(int(L//pas)):
                    
                    xk = x0 + k*pas*((x1-x0)/L)
                    yk = y0 + k*pas*((y1-y0)/L)
                    #print(f"x{i}{k} = {xk} ; y{i}{k} = {yk}")
                    
                    self.trajectory_pts[0].append(xk)   #x
                    self.trajectory_pts[1].append(yk)   #y
                    self.trajectory_pts[2].append(0)    #z || 0 position basse
            
            
            if i < len(self.lst_ensembles)-1:
                #lorque l'on passe d'un enseble à un autre il faut aussi créer le chemin avec z=1
                n0 = self.lst_ensembles[i][0] #On récupère le n° du point que l'on veut utiliser ici le premier
                n1 = self.lst_ensembles[i+1][0]
                
                print(f"n0 = {n0} ; n1 = {n1}")
                
                x0 = self.coords_peaks[n0][0]
                y0 = self.coords_peaks[n0][1]
                
                x1 = self.coords_peaks[n1][0]
                y1 = self.coords_peaks[n1][1]
                
                print(f"x0 = {x0} ; y0 = {y0}")
                print(f"x1 = {x1} ; y1 = {y1}")
                
                L = sqrt((x1-x0)**2+(y1-y0)**2)
                
                for k in range(int(L//pas)):
                    
                    xk = x0 + k*pas*((x1-x0)/L)
                    yk = y0 + k*pas*((y1-y0)/L)
                    #print(f"x{i}{k} = {xk} ; y{i}{k} = {yk}")
                    
                    self.trajectory_pts[0].append(xk)   #x
                    self.trajectory_pts[1].append(yk)   #y
                    self.trajectory_pts[2].append(1)    #z || 0 position basse
                
    def traj_d2r(self, fact_echelle = 10*(10**-2)):
                
                dim_px = fact_echelle / len(self.trajectory_pts[0])
                #print(f"dim px = {dim_px}")
                
                for i in range(len(self.trajectory_pts)-1):
                    for j in range(len(self.trajectory_pts[i])):
                        self.trajectory_pts_reel[i].append( self.trajectory_pts[i][j] * dim_px)
                        
    #regroupe toutes les fonctions précédentes     
    def image2coord(self,pas=1 ,fact_echelle = 10*(10**-2)):
        self.mapping_connexions()
        self.ensembles()
        self.trajectory_points(pas)
        self.traj_d2r(fact_echelle)