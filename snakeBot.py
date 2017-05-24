import sys,os
import pygame as pg
import random as r
from math import hypot
from math import pi
from math import sin
from math import cos
from math import atan2

##################################################--DynamicObstacle--###########################################################################

class DynamicObstacle:
    def __init__(self):
        self.randloc = r.randint(0,3)
        if (self.randloc == 0):
            self.rectangle = (r.randint(201,1300),5,r.randint(51,300),r.randint(31,150))
            self.dir_x = r.choice([-1,-1,-1,0,1,1,1])
            self.dir_y = -1
        if (self.randloc == 1):
            self.rectangle = (r.randint(0,1300),680,r.randint(51,300),r.randint(31,150))
            self.dir_x = r.choice([-1,-1,-1,0,1,1,1])
            self.dir_y = 1
        if (self.randloc == 2):
            self.rectangle = (5,r.randint(100,600),r.randint(51,300),r.randint(31,150))
            self.dir_x = 1
            self.dir_y = r.choice([-1,-1,-1,0,1,1,1])
        if (self.randloc == 3):
            self.rectangle = (610,r.randint(100,600),r.randint(51,300),r.randint(31,150))
            self.dir_x = -1
            self.dir_y = r.choice([-1,-1,-1,0,1,1,1])
        self.rect = pg.Rect(self.rectangle)
        self.obstImg = pg.Surface(self.rect.size).convert()
        self.obstImg.fill((0,255,0))
        self.invSpeed_x = r.randint(1,4)
        self.invSpeed_y = r.randint(1,4)
        self.isOut = False
    def mov(self,frames):
        if ((1530==self.rect.centerx)or(0==self.rect.centerx))and(frames%self.invSpeed_x == 0):
            self.dir_x = -self.dir_x
            self.isOut = True
        if ((770==self.rect.centery)or(0==self.rect.centery))and(frames%self.invSpeed_y == 0):
            self.dir_y = -self.dir_y
            self.isOut = True
        if (frames%self.invSpeed_x == 0):
            self.rect.move_ip(self.dir_x,0)
        if (frames%self.invSpeed_y == 0):
            self.rect.move_ip(0,self.dir_y)
    def draw(self,frames,surface):
        self.mov(frames)
        surface.blit(self.obstImg,self.rect)

###################################################--StaticObstacle--###########################################################################

class StaticObstacle:
    def __init__(self):
        self.rect = pg.Rect((r.randint(201,1300),r.randint(100,600),r.randint(51,200),r.randint(31,100)))
        self.obstImg = pg.Surface(self.rect.size).convert()
        self.obstImg.fill((0,255,0))
    def draw(self,_,surface):
        surface.blit(self.obstImg,self.rect)

########################################################--Snake--###############################################################################

class Snake:
    def __init__(self):
        self.pointlist = [(50,50)]
        # self.sinSnake = [(0,0)]*9
        # self.buffer = (50,50)
        self.a = pi/3
        self.b = 2*pi
        self.heading = None
        for i in range (1,9):
            self.pointlist.append((round(self.pointlist[i-1][0]+99*((1/8)*cos(self.a*cos(self.b*i/8)))),
                round(self.pointlist[i-1][1]+99*((1/8)*sin(self.a*cos(self.b*i/8))))))
        self.frameCount = i
        self.head= pg.Rect((0,0,6,6))
        self.head.center = self.pointlist[i]
        self.headImg = pg.Surface(self.head.size).convert()
        self.headImg.fill((255,0,0)) 
        self.moduleCenters = []
        self.sensorReading = []
        for i in range (0,8):
            moduleCenter_y = int((self.pointlist[i][1]+self.pointlist[i+1][1])/2)+1
            moduleCenter_x = int((self.pointlist[i][0]+self.pointlist[i+1][0])/2)+1
            self.moduleCenters.append((moduleCenter_x,moduleCenter_y))
    def mov(self,pos,frames):
        if (frames%9 == 0)and(self.distance(pos,self.head.center)>10):
            self.heading = round(atan2(pos[1]-self.head.centery,pos[0]-self.head.centerx),2)
            x=round(self.pointlist[len(self.pointlist)-1][0]+99*((1/8)*cos(self.a*cos(self.b*self.frameCount/8)+self.heading)))
            y=round(self.pointlist[len(self.pointlist)-1][1]+99*((1/8)*sin(self.a*cos(self.b*self.frameCount/8)+self.heading)))
            self.pointlist.append((x,y))
            self.pointlist = self.pointlist[1:]
            self.frameCount = self.frameCount+1
            self.moduleCenters = []
            for i in range (0,8):
                moduleCenter_y = int((self.pointlist[i][1]+self.pointlist[i+1][1])/2)+1
                moduleCenter_x = int((self.pointlist[i][0]+self.pointlist[i+1][0])/2)+1
                self.moduleCenters.append((moduleCenter_x,moduleCenter_y))
            self.head.center = self.pointlist[len(self.pointlist)-1]
        elif (frames%9==0)and(self.distance(pos,self.head.center)<=10):
            self.heading = None
    def draw(self,surface):
        pg.draw.lines(surface,(255,0,0),False,self.pointlist,8)
        surface.blit(self.headImg, self.head)
    def isAlive(self,obstacles):
        for obst in obstacles:
            for p in self.pointlist:
                if obst.rect.collidepoint(p):
                    return False
        return True
    def getSensorReading(self,surface,frames):
        if (frames%9 == 0):
            self.sensorReading = []
            for i in range (0,8):
                normal = self.getNormal(self.pointlist[i],self.pointlist[i+1])
                p = self.moduleCenters[i]
                while True:
                    p = (p[0]+normal[0],p[1]+normal[1])
                    if (p[0]>=1530)or(p[0]<=0)or(p[1]<=0)or(p[1]>=770):
                        self.sensorReading.append([9999])
                        break
                    elif (surface.get_at(p) == (0,255,0))or(surface.get_at(p) == (255,0,0)):
                        self.sensorReading.append([self.distance(self.moduleCenters[i],p)])
                        pg.draw.circle(surface,(0,0,255),p,1,0)
                        break
                    pg.draw.circle(surface,(0,0,255),p,1,0)
                normal = (-normal[0],-normal[1])
                p = self.moduleCenters[i]
                while True:
                    p = (p[0]+normal[0],p[1]+normal[1])
                    if (p[0]>=1530)or(p[0]<=0)or(p[1]<=0)or(p[1]>=770):
                        self.sensorReading[i].append(9999)
                        break
                    elif (surface.get_at(p) == (0,255,0))or(surface.get_at(p) == (255,0,0)):
                        self.sensorReading[i].append(self.distance(self.moduleCenters[i],p))
                        pg.draw.circle(surface,(0,0,255),p,1,0)                        
                        break
                    pg.draw.circle(surface,(0,0,255),p,1,0)
            print (self.sensorReading,self.heading),
    def distance(self,p1,p2):   return (int(hypot(p1[0]-p2[0],p1[1]-p2[1])))
    def getNormal(self,p1,p2):
        delta_x = p2[0]-p1[0]
        delta_y = p2[1]-p1[1]
        try:
            delta_x = int(8*(delta_x/hypot(delta_x,delta_y)))
            delta_y = int(8*(delta_y/hypot(delta_x,delta_y)))
        except:
            delta_x=0
            delta_y=0
        return (delta_y,-delta_x)

#######################################################--INITIALIZE--###########################################################################

os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (5,50)
pg.init()
pg.font.init()
clock = pg.time.Clock()
myfont = pg.font.SysFont('Comic Sans MS', 15)


size = width, height = 1530, 770
black = 0, 0, 0

screen = pg.display.set_mode(size)

snake = Snake()
latched = False

totalObstacles =  r.randint(5,7)
dynObstacles = r.randint(2,totalObstacles - 2)
statObstacles = totalObstacles - dynObstacles
dynObstList = []
statObstList = []
for i in range (0,dynObstacles):
    dynObst = DynamicObstacle()
    dynObstList.append(dynObst)
for i in range (0,statObstacles):
    statObst = StaticObstacle()
    statObstList.append(statObst)
obstacles = dynObstList + statObstList


frames = 0

##########################################################--LOOPING--###########################################################################

while 1:
    clock.tick(80)
    for event in pg.event.get():
        if event.type == pg.MOUSEBUTTONDOWN:
            if snake.head.collidepoint(event.pos):
                latched = True
        elif event.type == pg.MOUSEBUTTONUP:
            latched = False
        else:
            if event.type == pg.QUIT:
                sys.exit()

    textsurface = myfont.render('Time'+str(int(frames/150)), False, (255, 255, 255),(0,0,255))

    if latched:
        snake.mov(pg.mouse.get_pos(),frames)

    if snake.isAlive(obstacles):
        screen.fill(black)
        for obst in obstacles:
            obst.draw(frames,screen)
        snake.draw(screen)
        snake.getSensorReading(screen,frames)
        screen.blit(textsurface,(1400,700))
        frames = frames+1
    
    else:
        break

    pg.display.flip()
while 1:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            sys.exit()

    screen.fill(black)
    textsurface = myfont.render('You LOST. Your Score: '+str(int(frames/150)), False, (255, 255, 255),(0,0,255))
    screen.blit(textsurface,(765,335))
    pg.display.flip()
