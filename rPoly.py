import cv2
import numpy as np
import random as r

TRI = np.array([[-16,-16],[16,-16],[0,32]],np.float32)

def makeBorder(img):
    cv2.rectangle(img,(2,2),(1535,15),254,-1)
    cv2.rectangle(img,(2,704),(1535,717),254,-1)
    cv2.rectangle(img,(2,19),(15,701),254,-1)
    cv2.rectangle(img,(1522,19),(1535,701),254,-1)

def makeObst(img):
    cx = r.randint(270,1300)
    cy = r.randint(100,550)
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
    mask[markers == -1] = [155]
    mask = mask[20:699,16:1520]
    return mask

img = np.zeros((720,1800,3),np.uint8)

for i in range (0,r.randint(9,11)):
    makeObst(img)


mask = processMap(img)
cv2.imshow("mask",mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
