import cv2
import numpy as np
import random as r
from collections import defaultdict

TRI = np.array([[-16,-16],[16,-16],[0,32]],np.float32)
global gPaths
gPaths = []

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

def findNeighbors(r,c,img):
    neighbors = []
    flag = False
    if not(img.item(r-1,c-1) == 0):
        neighbors.append((c-1,r-1))
        flag = True
    if not(img.item(r-1,c) == 0):
        neighbors.append((c,r-1))
        flag = True
    if not(img.item(r-1,c+1) == 0):
        neighbors.append((c+1,r-1))
        flag = True
    if not(img.item(r,c-1) == 0):
        neighbors.append((c-1,r))
        flag = True
    if not(img.item(r,c+1) == 0):
        neighbors.append((c+1,r))
        flag = True
    if not(img[r+1][c-1] == 0):
        neighbors.append((c-1,r+1))
        flag = True
    if not(img.item(r+1,c) == 0):
        neighbors.append((c,r+1))
        flag = True
    if not(img.item(r+1,c+1) == 0):
        neighbors.append((c+1,r+1))
        flag = True
    if not(flag):
        neighbors.append((c,r))
    return neighbors

def removeDeadEnds(img,deadEnds):
    for (c,r) in deadEnds:
        while True:
            nodality = len(findNeighbors(r,c,img))
            if (img.item(r,c)==0):
                break
            if (nodality==2):
                neighbors = findNeighbors(r,c,img)
                cn,rn = neighbors[0] 
                if not(len(findNeighbors(rn,cn,img))==3):
                    break
            img.itemset((r,c),0)
            neighbors = findNeighbors(r,c,img)
            c,r = neighbors[0] 

def findNodes(img):
    nodes = []
    deadEnds = []
    for r in range (1,719):
        for c in range (1,1536):
            if not(img[r][c]==0):
                nodality = len(findNeighbors(r,c,img))
                if nodality<=1:
                    deadEnds.append((c,r))
                if nodality>=3:
                    if nodality>3:
                        print ("fuck up")
                    nodes.append((c,r))
    return nodes,deadEnds

def dist(p):
    return (p[0][0]**2+p[0][1]**2)
def dist2(p):
    return (p[0]**2+p[1]**2)

def processNodes(img,nodes):
    adjacency = []
    paths = []
    img,contours,_ = cv2.findContours(img,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)    
    cnt = contours[0]
    cnt = sorted(cnt,key=dist)
    start = tuple(cnt[0][0])
    end = tuple(cnt[len(cnt)-1][0])
    nodes.extend((start,end))
    nodes = sorted(nodes,key=dist2)
    for cnt in contours:
        lAdjacency = []
        lPaths = []
        path =[]
        pair = []
        c=1
        for p in cnt:
            if c==0:
                path.append((p[0][0],p[0][1]))
            if ((p[0][0],p[0][1]) in nodes):
                pair.append((p[0][0],p[0][1]))
                path.append((p[0][0],p[0][1]))
                lAdjacency.append(pair)
                lPaths.append(path)
                pair = [(p[0][0],p[0][1])]
                path = [(p[0][0],p[0][1])]
            c=(c+1)%70
        pth = lPaths[0]
        path.extend(pth)
        lPaths = lPaths[1:]
        lPaths.append(path)
        pr = lAdjacency[0][0]
        pair.append(pr)
        lAdjacency = lAdjacency[1:]
        lAdjacency.append(pair)
        adjacency.extend(lAdjacency)
        paths.extend(lPaths)
    nodePaths = paths
    adjacency,nodePaths = removeDuplicates(adjacency,nodePaths)    
    lAdjacency = []
    for a in adjacency:
        lAdjacency.append([a[1],a[0]])
    adjacency.extend(lAdjacency)
    adjacency = [tuple(pair) for pair in adjacency]
    adjacencyList = defaultdict(list)
    for k,v in adjacency:
        adjacencyList[k].append(v)
    return adjacencyList,nodePaths,nodes

def findNodePaths(adjacencyList,start,end,visited,path):
    visited[start]= True
    path.append(start)
    if start ==end:
        gPaths.append(tuple(path))
    else:
        for i in adjacencyList[start]:
            if visited[i]==False:
                findNodePaths(adjacencyList,i, end, visited, path)
    path.pop()
    visited[start]= False

def removeDuplicates(adj,npths):
    l = []
    adjacency=[]
    for [p1,p2] in adj:
        if not isNear(p1,l,True):
            l.append(p1)
        if not isNear(p2,l,True):
            l.append(p2)
    for [p1,p2] in adj:
        a =[]
        a.append(isNear(p1,l,False))
        a.append(isNear(p2,l,False))
        adjacency.append(a)
    nodePaths=[]
    for pth in npths:
        a =[]
        for pnt in pth:
            a.append(isNear(pnt,l,False))
        nodePaths.append(a)
    adjacency = [tuple(pair) for pair in adjacency]
    dct = dict(list(zip(adjacency,nodePaths)))
    l =[]
    for p in dct.keys():
        if (p[0]==p[1]):
            l.append(p)
    for a in l:
        dct.pop(a,None)
    adjacency = list(dct.keys())
    nodePaths = list(dct.values())
    return (adjacency,nodePaths)

def isNear(p,L,flag):
    for l in L:
        if (((l[0]-p[0])**2+(l[1]-p[1])**2)<6):
            if flag:
                return True
            else:
                return l
    if flag:
        return False
    else:
        return p

def getCurPath(p,nodePaths):
    for npth in nodePaths:
        if (p[0]==npth[0]) and (p[1]==npth[-1]):
            return npth
        elif (p[0]==npth[-1] and (p[1]==npth[0])):
            n=npth
            n.reverse()
            return n
    print ("Yo code is a sux")
    return None


obstMap = np.zeros((720,1537,3),np.uint8)
for i in range (0,r.randint(9,11)):
    makeObst(obstMap)
mask,paths,feasiblePaths = processMap(obstMap)
nodes, deadEnds = findNodes(feasiblePaths)
clearPaths = feasiblePaths.copy()
removeDeadEnds(clearPaths,deadEnds)
nodes,_ = findNodes(clearPaths)
adjacencyList,nodePaths,nodes = processNodes(clearPaths,nodes)
visited={}
for x in adjacencyList.keys():
    visited[x] = False
findNodePaths(adjacencyList,nodes[0],nodes[len(nodes)-1],visited,[])
gPaths = list(set(gPaths))
clearPaths = cv2.bitwise_or(clearPaths,mask)
for path in gPaths:
    new=clearPaths.copy()
    print("Path: ",path)
    for i in range(1,len(path)):
        pair = [path[i-1],path[i]]
        nodePath = getCurPath(pair,nodePaths)
        for pt in nodePath:
            cv2.circle(new,(pt[0],pt[1]),3,120,-1)
    cv2.imshow("nwe",new)
    cv2.waitKey()
    cv2.destroyAllWindows()
