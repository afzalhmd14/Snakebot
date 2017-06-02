import cv2
import numpy as np
import random as r

TRI = np.array([[-16,-16],[16,-16],[0,32]],np.float32)

def makeBorder(img):
    cv2.rectangle(img,(2,2),(1535,7),254,-1)
    cv2.rectangle(img,(2,712),(1535,717),254,-1)
    cv2.rectangle(img,(2,11),(7,708),254,-1)
    cv2.rectangle(img,(1530,11),(1535,708),254,-1)

def makeObst(img):
    cx = r.randint(270,1300)
    cy = r.randint(140,550)
    offset = np.array([[cx,cy],[cx,cy],[cx,cy]],np.float32)
    color = (r.randint(10,254),r.randint(10,254),r.randint(10,254))
    for i in range (0,50):
        scale = (r.randint(5,25))/10
        theta = np.radians(r.randint(15,180))
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c,-s],[s,c]],np.float32)
        mx = r.randint(0,35)
        my =r.randint(0,35)
        miniOff = np.array([[mx,my],[mx,my],[mx,my]],np.float32)
        new = scale*np.transpose(np.dot(R,np.transpose(TRI))) + offset + miniOff
        new = np.array([new],np.int32)
        cv2.fillPoly(img,new,color)

def processMap(img):
    WHITE = [254,254,254]
    kernel = np.ones((15,15),np.uint8)
    mask = cv2.inRange(img,np.array([2,2,2]),np.array([255,255,255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    makeBorder(mask)
    _, contours, hierarchy = cv2.findContours(mask.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    mask.astype(np.uint8)
    for cnt in contours:
        hull = cv2.convexHull(cnt)
        cv2.fillConvexPoly(mask,hull,(255,255,255))
    _, sure_fg = cv2.threshold(mask,127,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    _, markers = cv2.connectedComponents(np.uint8(sure_fg))
    markers = cv2.watershed(img,markers)
    paths = mask.copy()
    paths[markers == -1] = [155]
    dt = cv2.distanceTransform(cv2.bitwise_not(mask),cv2.DIST_L2,5)
    _,dt = cv2.threshold(dt,0.1*dt.max(),255,0)
    dt = np.uint8(dt)
    feasiblePaths = cv2.bitwise_and(paths, dt)
    return mask,paths,feasiblePaths

def checkNodality(r,c,img):
    nodality = 0
    if not(img.item(r-1,c-1) == 0):
        nodality = nodality+1
    if not(img.item(r-1,c) == 0):
        nodality = nodality+1
    if not(img.item(r-1,c+1) == 0):
        nodality = nodality+1
    if not(img.item(r,c-1) == 0):
        nodality = nodality+1
    if not(img.item(r,c+1) == 0):
        nodality = nodality+1
    if not(img.item(r+1,c-1) == 0):
        nodality = nodality+1
    if not(img.item(r+1,c) == 0):
        nodality = nodality+1
    if not(img.item(r+1,c+1) == 0):
        nodality = nodality+1
    return nodality

def findNeighbor(r,c,img):
    if not(img.item(r-1,c-1) == 0):
        return (c-1,r-1)
    if not(img.item(r-1,c) == 0):
        return (c,r-1)
    if not(img.item(r-1,c+1) == 0):
        return (c+1,r-1)
    if not(img.item(r,c-1) == 0):
        return (c-1,r)
    if not(img.item(r,c+1) == 0):
        return (c+1,r)
    if not(img[r+1][c-1] == 0):
        return (c-1,r+1)
    if not(img.item(r+1,c) == 0):
        return (c,r+1)
    if not(img.item(r+1,c+1) == 0):
        return (c+1,r+1)
    else:
        return (c,r)

def removeDeadEnds(img,deadEnds):
    for (c,r) in deadEnds:
        while True:
            nodality = checkNodality(r,c,img)
            if (img.item(r,c)==0):
                break
            if (nodality==2):
                cn,rn = findNeighbor(r,c,img)
                if not(checkNodality(rn,cn,img)==3):
                    break
            img.itemset((r,c),0)
            c,r = findNeighbor(r,c,img)

def findNodes(img):
    nodes = []
    deadEnds = []
    for r in range (1,719):
        for c in range (1,1536):
            if not(img[r][c]==0):
                nodality = checkNodality(r,c,img)
                if nodality<=1:
                    deadEnds.append((c,r))
                if nodality>=3:
                    flag =1
                    for n in nodes:
                        if (((c-n[0])**2+(r-n[1])**2)<10):
                            flag = 0
                            break
                    if flag:
                        nodes.append((c,r))
    return nodes,deadEnds

img = np.zeros((720,1537,3),np.uint8)

for i in range (0,r.randint(9,11)):
    makeObst(img)

mask,paths,feasiblePaths = processMap(img)
nodes,deadEnds = findNodes(feasiblePaths)
clearPaths = feasiblePaths.copy()
removeDeadEnds(clearPaths,deadEnds)
cv2.imshow("Feasable Paths",feasiblePaths)
cv2.waitKey()
cv2.imshow("Clear Paths",clearPaths)
cv2.waitKey()
cv2.destroyAllWindows()
