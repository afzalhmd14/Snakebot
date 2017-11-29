import vrep
import csv
from math import sin
from math import acos
from math import pi
from math import sqrt
from time import sleep
import random as r
import numpy as np
import cv2
from rPoly import rPoly
#--------------------------------------------Goal values
x = -6

y = +7
#--------------------------------------------Initialise simulation paramemters

dt = 0.1  #---------------------------------Simulation time-step
realcount = 0 #------------------------------Real time counter
#--------------------------------------------Snake class

class Snake:
#--------------------------------------------Initialise centroid of snake
    centroid = [-9.5, 10.0]
    lastcentroid = [0, 0]
#--------------------------------------------Snake physical parameters
    n = 8
    a = pi/6
    b = 3*pi
    c = 0
#--------------------------------------------Snake joint parameters
    beta = b/n
    alpha = 2*a*sin(beta/2)
    gamma =[-c/n,-c/n,-c/n,-c/n,-c/n,-c/n,-c/n,-c/n]
    omega = -2
#--------------------------------------------Init the handles
    def __init__(self,clientID):

        motors =['motor1','motor2','motor3','motor4','motor5','motor6','motor7']
        self.motor_handles = [vrep.simxGetObjectHandle(clientID,name, vrep.simx_opmode_blocking)[1] for name in motors]

        sensors = ['head_sensor','sensor1r','sensor1l','sensor2r','sensor2l','sensor3r','sensor3l','sensor4r','sensor4l',	'sensor5r','sensor5l','sensor6r','sensor6l','sensor7r','sensor7l','sensor8r','sensor8l']
        self.sensor_handles = [vrep.simxGetObjectHandle(clientID, sensor, vrep.simx_opmode_blocking)[1] for sensor in sensors]

        obstacles = ['Obj0','Obj1','Obj2','Obj3','Obj4','Obj5','Obj6','Obj7','Obj8','Obj9','Obj10']
        self.obst_handles = [vrep.simxGetObjectHandle(clientID, obj, vrep.simx_opmode_blocking)[1] for obj in obstacles]

        body_modules = ['snake_v2', 'module2', 'module3', 'module4', 'module5', 'module6', 'module7', 'module8']
        self.body_module_handles = [vrep.simxGetObjectHandle(clientID, obj, vrep.simx_opmode_blocking)[1] for obj in body_modules]

        self.snake_body = vrep.simxGetObjectHandle(clientID, 'snake_v2', vrep.simx_opmode_blocking)[1]

        print ("\nSet motor torques...\n")
        for motor_handle in self.motor_handles:
            _=vrep.simxSetJointForce(clientID,motor_handle,5,vrep.simx_opmode_blocking)
            if _ !=0: raise Exception()

        self.setMotor(clientID, 0, 1)


        print ("\nAssign sensor values\n")
        for sensor_handle in self.sensor_handles:
            _,_,_,_,_ = vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)


        print ("\nSetting Object postions...\n")
        self.obst_count = r.randint(5,8)
        for i in range (self.obst_count):
            obst = r.choice(self.obst_handles)
            _, [obst_copy] = vrep.simxCopyPasteObjects(clientID, [obst], vrep.simx_opmode_blocking)
            x = round(r.uniform(-5,8),3)
            y = round(r.uniform(-7.5,8),3)
            _ = vrep.simxSetObjectPosition(clientID, obst_copy, -1, (x, y, 0.25),vrep.simx_opmode_oneshot)
            _ = vrep.simxSetObjectOrientation(clientID, obst_copy, obst_copy, (r.randint(0,180),0,0), vrep.simx_opmode_oneshot)

        #_ = vrep.simxSetObjectPosition(clientID, self.snake_body, -1, (-9.5, 10, 0.085), vrep.simx_opmode_oneshot)


        #---------------------------------------------Initialising the motor position buffer
        self.modulepos = list()
        for motor_handle in self.motor_handles:
            _,moduleCord = vrep.simxGetObjectPosition(clientID,motor_handle,-1,vrep.simx_opmode_streaming)
            self.modulepos.append(moduleCord)
            if _ !=1 :
                print (_)
                raise Exception()

        #---------------------------------------------Initialising body_modules collision buffer
        for module in self.body_module_handles:
            _,collision_state = vrep.simxReadCollision(clientID, module, vrep.simx_opmode_streaming)
            print _

        #print modulepos\

    def updatePosition(self,clientID):

        self.lastcentroid = self.centroid

        for j in range(0,len(self.modulepos)):

            _,moduleCord = vrep.simxGetObjectPosition(clientID,self.motor_handles[j],-1,vrep.simx_opmode_buffer)
            self.modulepos.pop()
            self.modulepos.insert(j,moduleCord)
            #if _ !=1:

            #	print (_)
            #	raise Exception()

            #print j

        xcentroid=0
        ycentroid=0
        j=0

        for j in range(0,len(self.modulepos)):
            xcentroid = self.modulepos[j][0] + xcentroid
            ycentroid = self.modulepos[j][1] + ycentroid

        xcentroid = xcentroid/self.n
        ycentroid = ycentroid/self.n

        self.centroid = [xcentroid,ycentroid]

        # print ("Previous x = ",self.lastcentroid[0])
        #
        # print ("Previous y = ",self.lastcentroid[1])
        #
        # print ("Present x = ",self.centroid[0])
        #
        # print ("Present y = ",self.centroid[1])
        #
        # print ('\n')


    def updateDirection(self,clientID,x,y,dist):

        #print ("Changing course ...")
    #--------------------------------------------Making vectors to find dot product

        vector1 =[x-self.centroid[0],y-self.centroid[1]]
        vector2 =[self.lastcentroid[0]-self.centroid[0],self.lastcentroid[1]-self.centroid[1]]

    #-----------------------------------------FINDING DIRECTION OF GOAL POINT WRT TO SNAKE- left or right
    #-----------------------------------------positive c value is for left turn and negative for right turn

        direction =(x- self.lastcentroid[0])*(self.centroid[1]- self.lastcentroid[1]) - (y - self.lastcentroid[1])*(self.centroid[0]- self.lastcentroid[0])
        direction = -1*direction/abs(direction)

        # print (direction)
    #-----------------------------------------Assiging c value to snake

    #-----------------------------------------Set range of c to (-4,4)
        self.c = max( min( 5, 2.0* (1+3.2*pow(2.71,-0.5*dist))*direction*(pi - acos(np.dot(vector2,vector1)/(np.linalg.norm(vector1)*np.linalg.norm(vector2)))) ), -5 )
        # print ("new c: ", self.c)
        print ('Turning by :' , self.c)
        # print ('\n')
        # _,head = vrep.simxGetObjectPosition(clientID,self.motor_handles[0],-1,vrep.simx_opmode_buffer)
        # ret = np.linalg.norm([x-head[0],y-head[1]])
        # print (ret)
        # return ret

        # ---------------------------------------Set the head to new c value, orientes snake direction
        self.gamma[0] = -self.c / self.n

    def setMotor(self,clientID,count,jointNo):

        for motor_handle in self.motor_handles:

            phi = self.alpha*sin(self.omega*count + (jointNo-1)*self.beta) + self.gamma[jointNo-1]

            _=vrep.simxSetJointTargetPosition(clientID,motor_handle,phi,vrep.simx_opmode_blocking)
            if _ !=0 :
                print (_)
                raise Exception()
            #sleep(0.001)
            jointNo+=1

    def readSensor(self):
        new_sensor_reading = []
        for sensor_handle in self.sensor_handles:
            error, state, point, _, _ = vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)
            if not state:
                new_sensor_reading.append(11.)
            else:
                reading = sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                new_sensor_reading.append(reading)

        return new_sensor_reading

        #print (count, new_sensor_reading )

    def dist2target (self,clientID,x,y):
        _,head = vrep.simxGetObjectPosition(clientID,self.motor_handles[0],-1,vrep.simx_opmode_buffer)
        ret = np.linalg.norm([x-head[0],y-head[1]])
        return ret

# -------------------------------------------------To save the sensor data in .csv file
def createNewCSV(iter):
	filename = "data-"+str(iter)+".csv"
	writer = csv.writer(open(("C:/Users/Afzal/Desktop/Snakebot/sensor_data/"+filename),'w'))
	return writer

#-----------------------------------------------------Run through several iterations

if __name__ == "__main__":


    iteration = 50


    while(iteration <= 200):

        # close any open connections
        vrep.simxFinish(-1)
        # Connect to the V-REP continuous server
        clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)

        if clientID != -1:
            # -----------------------------------------------if we connected successfully
            print ('Connected to remote API server')

            # ---------------------------------------------- Setup the simulation
            vrep.simxSynchronous(clientID, True)
            # -----------------------------------------------move simulation ahead one time step
            vrep.simxSynchronousTrigger(clientID)

            print ("Simulation Starts...\n")

            # ------------------------------------------------Set the simulation time step as flaot
            vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, dt,
                                          vrep.simx_opmode_oneshot)

            # ---------------------------------------------- Start the simulation

            # ------------------------------------------------Start our simulation in lockstep with our code
            vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

            # --------------------------------------------Initialise the snake and it's handles
            snake = Snake(clientID)
            top_view = vrep.simxGetObjectHandle(clientID, 'top_view', vrep.simx_opmode_blocking)[1]
            disc = vrep.simxGetObjectHandle(clientID, 'Disc', vrep.simx_opmode_blocking)[1]
            black_plane = vrep.simxGetObjectHandle(clientID, 'Plane', vrep.simx_opmode_blocking)[1]

            #-------------------------------------------------Reposition the objects and snake
            #snake.reposition(clientID)

            # ---------------------------------------------------------bring back the black plane
            #_ = vrep.simxSetObjectPosition(clientID, black_plane, -1, (0, 0, 0.2), vrep.simx_opmode_oneshot)


            #------------------------------------------------Initialise the .csv file for data storage
            data_writer =createNewCSV(iteration)

            #------------------------------------------------Get the top view of arena

            vrep.simxSynchronousTrigger(clientID)

            _, res, img = vrep.simxGetVisionSensorImage(clientID, top_view, 1, vrep.simx_opmode_blocking)
            print (res)
            sleep(3)

            img = np.array(img,dtype = np.uint8)
            img.resize([res[0],res[1]])
            img = cv2.flip(img,0)

            cv2.imshow("img",img)
            cv2.waitKey()
            cv2.destroyAllWindows()

            paths = rPoly(img)
            vrep_paths = []
            for path in paths:
                vrep_path = []
                for point in path:
                    vrep_point = (point[0] * 25. / 512. - 12.5, (511 - point[1]) * 25. / 512. - 12.5)
                    vrep_path.append(vrep_point)
                vrep_paths.append(vrep_path)

            path = vrep_paths[0]
            _, [disc_copy] = vrep.simxCopyPasteObjects(clientID, [disc], vrep.simx_opmode_blocking)
            _ = vrep.simxSetObjectPosition(clientID, disc_copy, -1,(path[0][0], path[0][1], 1), vrep.simx_opmode_oneshot)

            #---------------------------------------------------------testing remove plane to see the snake
            _ = vrep.simxSetObjectPosition(clientID, black_plane, -1, (0,0,-2), vrep.simx_opmode_oneshot)

            n=0 #------------------------------------------index of next point on path
            ret =1000#--------------------------------------return value of dist2disc
            count = 0.0  # --------------------------------Simulation time counter



            while count<5000 and n< (len(path)-1):

                #----------------------------------------------Reading collision status
                # collision = False
                # for module in snake.body_module_handles:
                #     _,collision = vrep.simxReadCollision(clientID, module, vrep.simx_opmode_buffer)
                #     #print collision
                #     if _ != 1:
                #         print (_)
                #         #raise Exception()
                #     if collision == True:
                #         break

                #----------------------------------------------Read the body sensors
                sensor_data = snake.readSensor()
                sensor_data.append(snake.c)
                data_writer.writerow(sensor_data)
                #----------------------------------------------check if snake is rightly oriented

                #if realcount%snake.n==0 and realcount!=0:#-------------check only after the all joints have traversed

                #-------Take centroid of the entire snake body and aligne it with the target point --colineraity check
                snake.updatePosition(clientID)
                    #snake.updateDirection(clientID,path[n][0],path[n][1])

                ret = snake.dist2target(clientID,path[n][0],path[n][1])
                if (ret <1.7) and (n < len(path)-1):
                    n += 1
                    print("NEXT POINT ", path[n])
                    _ = vrep.simxSetObjectPosition(clientID, disc_copy, -1,(path[n][0], path[n][1], 1), vrep.simx_opmode_oneshot)
                    # if n== len(path)-1:
                    #     break


                snake.updateDirection(clientID, path[n][0], path[n][1],ret)

                #-----------------------------------------------Set motor angles to move
                snake.setMotor(clientID,count,1)
                i=7
                while i>0:
                    snake.gamma[i]=snake.gamma[i-1]
                    i-=1
                #----------------------------------------------Trigger fo next step
                vrep.simxSynchronousTrigger(clientID)
                count += dt
                count = round(count,1)

                #----------------------------------------------Real time counter
                realcount +=1

            # -------------------------------------------------Stop the simulation
            _=vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
            if _ !=0 : raise Exception()

            vrep.simxFinish(clientID)

        else:
            print ('Failed connecting to remote API server')


        sleep(.1)
        iteration += 1

    print ('\nProgram ended')