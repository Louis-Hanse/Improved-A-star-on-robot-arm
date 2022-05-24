import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from time import sleep
 
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
lp = 0
distLog = 0
count = 0
while True:
    lpCache = lp
    try:
        log = open('log.txt','r')
        xs = eval(log.readline())
        ys = eval(log.readline())
        zs = eval(log.readline())
        dist = eval(log.readline())
        lp = eval(log.readline())
        distLog = distLog + dist
        count += 1
        average = distLog/count
        log.close()
    except:
        continue
    if lp <= lpCache:
        continue
    print(round(xs,4),end='\t')
    print(round(ys,4),end='\t')
    print(round(zs,4),end='\t')
    print(round(dist,4),end='\t')
    print(lp,end='\t')
    print(round(average,4))
    #ax.scatter(xs, ys, zs, c='r', marker='o')
    #plt.show()
    sleep(0.25)