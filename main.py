from class_graph import *
import matplotlib.pyplot as plt

#%%
#déclaration de l'élément graph de la classe graph
graph1 = graph("TE.png")

#%% vhvghvgh
#On test que la fonction fonctionne bien dans la classe
#graph1.connect2points(6,9) #ça fonctionne

#On test la fonction mapping
graph1.mapping_connexions()
print("lst connections : ")
print(graph1.lst_connections)

graph1.ensembles()
print("lst_ensembles")
print(graph1.lst_ensembles)

graph1.trajectory_points(1)

graph1.traj_d2r()

"""
plt.figure(0)
plt.plot(graph1.trajectory_pts[0],graph1.trajectory_pts[1])
plt.show()
"""
plt.figure(1)
plt.plot(graph1.trajectory_pts_reel[0],graph1.trajectory_pts_reel[1])
plt.show()


plt.figure(2)
plt.plot(graph1.trajectory_pts[0])
plt.show()

plt.figure(3)
plt.plot(graph1.trajectory_pts[1])
plt.show()

plt.figure(4)
plt.plot(graph1.trajectory_pts[2])
plt.show()

#%%

        
        
    
    
    


