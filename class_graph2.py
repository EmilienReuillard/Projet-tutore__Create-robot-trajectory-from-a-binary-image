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
import skimage.color as color

def inv_x_y(lst):
    L = len(lst) 
    for i in range(L):
        var = lst[i][0]
        lst[i][0] = lst[i][1]
        lst[i][1] = var
        
def lst_num(pt1,pt2,pas):
        
        res = [ [], [], []]
        
        D = sqrt((pt1[0] - pt2[0])**2 + 
                 (pt1[1] - pt2[1])**2 )
                    
        for k in range(int(D//pas)):
            
            xk = pt1[0] + k*pas*( ( pt2[0] - pt1[0] ) / D )
            yk = pt1[1] + k*pas*( ( pt2[1] - pt1[1] ) / D )
            zk = 0
            
            res[0].append(int(xk))
            res[1].append(int(yk))
            res[2].append(int(zk))
            
        return res
        

class Ensemble:
    
    min_X = 0
    min_Y = 0
    N_pt_canny = 0  #Nombre max d'itération pour la suite
    
    lst_tot = []    #X et Y
    
    
    
    def __init__(self, N_ensemble, graph, rayon = 10):
        self.new_img = graph.new_image_from_single_ensemble(N_ensemble)
        self.new_img_blur = skimage.filters.gaussian(self.new_img, (1,1))
        self.new_img_canny = canny(self.new_img, 1)
        self.new_img_canny_blur = skimage.filters.gaussian(self.new_img_canny, (1,1))
        self.Lx = len(self.new_img_canny)
        self.Ly = len(self.new_img_canny[0])
        self.rayon = rayon
        self.N_ensemble = N_ensemble
        
        #Détection des coins dans l'image
        self.coords_peaks = corner_peaks(corner_harris(self.new_img_blur), min_distance=5, threshold_rel=0.02)
        self.N_peaks = len(self.coords_peaks)
        
        #dans coord_peaks, x et y sont inversés, on va donc les remettre dans le bon ordre
        inv_x_y(self.coords_peaks)
        
    #prends en argument un canny (pas un blur)
    def rech_min_pt(self):
        lst_pt_canny = [[],[]]  #liste qui regroupe tous les points [x,y] de new_image1_canny
        
        for i in range(self.Lx):
            for j in range(self.Ly):
                if self.new_img_canny[i][j] == 1:
                    lst_pt_canny[0].append(i)
                    lst_pt_canny[1].append(j)
                    self.N_pt_canny += 1
                    
        self.min_X = min(lst_pt_canny[0])           #minX
        self.min_Y = min(lst_pt_canny[1])           #minY
        self.first_pt = [self.min_X , self.min_Y]   #Coord du point de départ de la recherche
        self.last_pt1 = self.first_pt    #On définit ici la variable "last_pt1"
        self.last_pt2 = self.last_pt1
        

    def rech_pt_cadrillage(self,pas = 1, seuil = 0.1):
        #Lx, Ly sont les dimentions de l'image
        #on vas parcourrir l'image pour détecter des points
        
        self.pas = pas
        
        self.N_detect = 0 
        
        #liste qui va regrouper les points détectés
        self.lst_detect = []   
                
        #détection
        for i in range(self.Lx//pas):
            for j in range(self.Ly//pas):
                
                i_pas = int(i*pas)
                j_pas = int(j*pas)
                
                if( self.new_img_canny[i_pas][j_pas] > seuil ):
                    self.N_detect += 1
                    x = j_pas
                    y = i_pas 
                    var = [x,y]
                    self.lst_detect.append(var)
        
        
                    
    def point_le_plus_proche(self,pt1):
        
        pt_le_plus_proche = 0 
        Dist_min = 10000000 #grande valeure arbitraire
        
        for pt2 in range(self.N_detect):
            
            Dist = sqrt((self.lst_detect[pt1][0] - self.lst_detect[pt2][0])**2 + 
                        (self.lst_detect[pt1][1] - self.lst_detect[pt2][1])**2 )
            
            if ((Dist <= Dist_min) and (pt1 != pt2) and (pt2 not in self.used_points)):
                pt_le_plus_proche = pt2
                Dist_min = Dist
                
        return pt_le_plus_proche, Dist_min
    
    #on utilise la fonction précédente pour mettre les points dans l'ordre
    def mapping(self):
        
        #Trouver le minimum à partir de lst_detect et trouver l'indice également
        #trouver le X min, puis le Y min sur cette ligne de X
        
        self.min_lst_detect = min(self.lst_detect)
        self.ind_min_lst_detect = self.lst_detect.index(self.min_lst_detect)
        
        
        #on crée la liste qui va reçecoir tous les points dans le bon ordre
        self.lst_trajectory = []
        #liste qui répertorie les points déjà utilisés
        self.used_points = []
        
        #Point de départ
        self.first_pt = [self.min_lst_detect[0], self.min_lst_detect[1] , 0 ,  self.ind_min_lst_detect ]
        self.lst_trajectory.append(self.first_pt)    #ajout du 1er point
        self.used_points.append(self.ind_min_lst_detect)
        
        #on va maintenan aller de point le plus proche en point le plus proche
        compt = 0

        
        for i in range(self.N_detect - 1):
            
            
            ind_new_pt, D = self.point_le_plus_proche(self.lst_trajectory[compt][3])  #on trouve l'indice du point le plus proche
            
            if D < 50: 
                self.used_points.append(ind_new_pt)
                new_pt = [ self.lst_detect[ind_new_pt][0], self.lst_detect[ind_new_pt][1], 0 , ind_new_pt] 
                self.lst_trajectory.append(new_pt)
                compt += 1
            
                
        #Eliminiation des erreures (gros changement de dirrection)
        #On crée des points intermédiares A FAIRE POUR LA PROCHAINE FOIS
        
        #on repasse toute la liste en revue
        for i in range(compt):
            
            #point 1 ; point 2
            pt1 = self.lst_trajectory[i]
            pt2 = self.lst_trajectory[i+1]
            
            #Distance entre les 2 points
            D = sqrt((pt1[0] - pt2[0])**2 + 
                     (pt1[1] - pt2[1])**2 )

            #si l'espace entre 2 points est trop grands, on le comble
            #ATTENTION !!!!!
            #Les points ajoutés n'ont pas de troisième composante qui pointe vers le numéro du prochain point
            
            if D > 10*self.pas:
                
                n_pt_aj = 0 #nombre de points ajoutés
                for k in range(int(D//self.pas)):
                    
                    xk = pt1[0] + k*self.pas*( ( pt2[0] - pt1[0] ) / D )
                    yk = pt1[1] + k*self.pas*( ( pt2[1] - pt1[1] ) / D )
                    
                    var = [int(xk), int(yk) , 0]
                    self.lst_trajectory.insert(i+k+1, var)
                    n_pt_aj += 1
                
                
        self.last_pt = self.lst_trajectory[-1]    #dernier point a avoir été tracé
            
    #Synthèse de toutes les fonctions précédentes
    def mapping_process(self, pas=1):
        self.rech_pt_cadrillage(pas)
        self.mapping()
        
    def affiche_X_Y(self):
        lstx = []
        lsty = []

        for i in range(len(self.lst_trajectory)):
            lstx.append(self.lst_trajectory[i][0])
            lsty.append(self.lst_trajectory[i][1])
  
        #plt.plot(lstx,lsty)
        #plt.show()

        #Affichage
        nrows = 1
        ncols = 2

        fig = plt.figure(figsize=(10, 5))
        fig.subplots(nrows, ncols)

        fig.add_subplot(nrows, ncols, 1)
        plt.imshow(self.new_img_canny_blur,cmap='gray')
        plt.axis('off')
        plt.title("Image canny Blur")

        fig.add_subplot(nrows, ncols, 2)
        plt.plot(lstx,lsty)
        plt.axis('off')
        plt.title("Image de base en niveau de gris")

        plt.show()


class Graph2:
    
#_______VARIABLES_______   

    lst_connections = []    #Pour chaque pic, répértorie les points connectés au n-ieme de la liste
    lst_ensembles = []  #Répertorie les points interconnerctés
    lst_tot = [ [] , [] , [] ]  #liste qui fait le total de toutes les coordonnées trouvées précedement [x,y,z]
    trajectory_pts_reel = [[],[],[]] #coordonnées réelles à parir d'un facteur d'échelle
    
#______FONCTIONS_____

    def __init__(self,path_image):
        #ouverture de l'image
        self.image_rgb = io.imread(path_image)
        self.image_gray = color.rgb2gray(self.image_rgb)
        self.image_canny = canny(self.image_gray,1)
        self.image_canny_blur = skimage.filters.gaussian(self.image_canny, (1,1))
        
        #definition des constante de taille
        self.Lx = len(self.image_rgb)
        self.Ly = len(self.image_rgb[0])
        
    #on isole une seul région n°"indice_region" en créant une nouvelle image
    #Cette image sera utilisée par la suite pour faire des traitements
    def new_image_from_single_ensemble(self, indice_region):
        new_image = np.zeros((self.Lx, self.Ly))   #nouvelle image pour chaque région
        region = self.props[indice_region]
        
        #remplissage de new_image
        for (x,y) in region.coords :
            new_image[x][y] = 1
            
        return new_image
        
    def affiche_imgs(self):
        plt.imshow(self.image_rgb)
        plt.show()
        plt.imshow(self.image_gray,cmap="gray")
        plt.show()
        plt.imshow(self.image_canny,cmap="gray")
        plt.show()
        plt.imshow(self.image_canny_blur,cmap="gray")
        plt.show()
        
    def find_ensembles(self):
        #trouver les différents ensembles
        self.Label = label(self.image_gray)
        self.props = regionprops(self.Label)
        self.N_regions = len(self.props)
        
    def createAllEnsembles(self, pas = 1):
        
        self.pas_graph = pas
        
        for i in range(self.N_regions):
            
            E = Ensemble(i, self)
            E.mapping_process(pas)
            self.lst_ensembles.append(E)
            
            self.lst_ensembles[i].affiche_X_Y()
        
    def connectAllEnsemble(self):
        
        for i in range(self.N_regions):
            for j in range(len(self.lst_ensembles[i].lst_trajectory)):
                self.lst_tot[0].append(self.lst_ensembles[i].lst_trajectory[j][0])
                self.lst_tot[1].append(self.lst_ensembles[i].lst_trajectory[j][1])
                self.lst_tot[2].append(self.lst_ensembles[i].lst_trajectory[j][2])
                
            #Ajout du chemin pour le prochain ensemble
            if i < self.N_regions - 1:
                pt1 = self.lst_ensembles[i].last_pt
                pt2 = self.lst_ensembles[i+1].first_pt
                
                #Distance entre les 2 points
                D = sqrt((pt1[0] - pt2[0])**2 + 
                         (pt1[1] - pt2[1])**2 )
                    
                for k in range(int(D//self.lst_ensembles[i].pas)):
                    
                    xk = pt1[0] + k*self.lst_ensembles[i].pas*( ( pt2[0] - pt1[0] ) / D )
                    yk = pt1[1] + k*self.lst_ensembles[i].pas*( ( pt2[1] - pt1[1] ) / D )
                    zk = 1
                    
                    self.lst_tot[0].append(int(xk))
                    self.lst_tot[1].append(int(yk))
                    self.lst_tot[2].append(int(zk))
                    
     
        
    #Mettre le repère en bas à gauche           
    def RotationRep(self):
        #mettre l'origine en bas à gauche, actuellement on est en haut à gauche
        
        new_listx = self.lst_tot[0]
        new_listy = []
            
        for i in range(len(new_listx)):
            new_listy.append( self.lst_tot[1][i]*-1.0 + max(self.lst_tot[1]))
                
        self.lst_tot = [new_listx, new_listy, self.lst_tot[2]]
        
        
    
        
    
    def ajout_cadre(self):
        
        Xmin = int(min(self.lst_tot[0]))
        Xmax = int(max(self.lst_tot[0]))
        Ymin = int(min(self.lst_tot[1]))
        Ymax = int(max(self.lst_tot[1]))
        
        pt1 = [Xmin,Ymin]
        pt2 = [Xmax,Ymin]
        pt3 = [Xmax,Ymax]
        pt4 = [Xmin,Ymax]
        
        lst1 = lst_num(pt1,pt2,self.pas_graph)
        lst2 = lst_num(pt2,pt3,self.pas_graph)
        lst3 = lst_num(pt3,pt4,self.pas_graph)
        lst4 = lst_num(pt4,pt1,self.pas_graph)
        
        lst = [lst1,lst2,lst3,lst4]
        
        for i in range(len(lst)):
            for j in range(len(lst[0])):
                for k in range(len(lst[0][0])):
                    self.lst_tot[j].append(lst[i][j][k])
    
    
    
    def traj_d2r(self, fact_echelle = 10*(10**-2)): #fact_echelle == longueur en x de l'image réelle
        
        dim_px = fact_echelle / max(self.lst_tot[0])
        
        #calcul de la longeur de l'image en y
        self.dim_reel_y = dim_px*max(self.lst_tot[1])    #On multiplie la dimention d'un px avec la 'taille' 
        
        for i in range(len(self.lst_tot)):
            for j in range(len(self.lst_tot[i])):
                self.trajectory_pts_reel[i].append( self.lst_tot[i][j] * dim_px)
        
    
    #Affichage final
    def Affiche_Final(self):
        plt.plot(self.lst_tot[0],self.lst_tot[1])
        plt.show()
    
    def ProcessingGene(self, pas = 1, fact_echelle = 1,affichage = 0 ,cadre = 0):
        self.find_ensembles()
        self.createAllEnsembles(pas)
        self.connectAllEnsemble()
        self.RotationRep()
        
        if cadre == 1:
            self.ajout_cadre()
            
        self.traj_d2r(fact_echelle)
        
        if affichage == 1:
            self.Affiche_Final()