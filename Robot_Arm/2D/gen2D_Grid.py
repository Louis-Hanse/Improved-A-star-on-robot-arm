#gen2D_Grid
import os
from PIL import Image

path = 'F:\\SyncDocuments\\毕业论文\代码\\Robot_Arm\\2D\\'
os.chdir(path)
gridImg = Image.open('map.png')
size = gridImg.size
print(size)

print('------------start------------')

for i in range(size[0]):
    print("'",end='')
    for j in range(size[1]):
        if gridImg.getpixel((i,size[1]-j-1)) == (255,255,255,255):
            #empty
            print("o",end='')
        elif gridImg.getpixel((i,size[1]-j-1)) == (0,0,0,255):
            #block
            print("x",end='')
        elif gridImg.getpixel((i,size[1]-j-1)) == (255,0,0,255):
            #start
            print("s",end='')
        elif gridImg.getpixel((i,size[1]-j-1)) == (0,255,0,255):
            #goal
            print("g",end='')
        else:
            continue
    print("';")

print('-------------end-------------')