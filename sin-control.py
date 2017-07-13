import vrep
from math import sin
from math import pi
from math import sqrt
from time import sleep
import threading
import random as r
import numpy as np
import cv2
from rPoly import rPoly

# close any open connections
vrep.simxFinish(-1)
# Connect to the V-REP continuous server
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)
change = False


def update_direction():
    global no_input, change, c
    c = float(input("\nneed input: "))
    change = True


if clientID != -1:  # if we connected successfully
    print('Connected to remote API server')

    # --------------------- Setup the simulation
    vrep.simxSynchronous(clientID, True)
    # move simulation ahead one time step
    vrep.simxSynchronousTrigger(clientID)

    dt = .1
    vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, dt, vrep.simx_opmode_oneshot)

    n = 8
    a = pi / 5
    b = 1.5 * pi
    c = 0

    motors = ['motor1', 'motor2', 'motor3', 'motor4', 'motor5', 'motor6', 'motor7']
    motor_handles = [vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)[1] for name in motors]

    sensors = ['sensor1r', 'sensor1l', 'sensor2r', 'sensor2l', 'sensor3r', 'sensor3l', 'sensor4r', 'sensor4l',
               'sensor5r', 'sensor5l', 'sensor6r', 'sensor6l', 'sensor7r', 'sensor7l', 'sensor8r', 'sensor8l']
    sensor_handles = [vrep.simxGetObjectHandle(clientID, sensor, vrep.simx_opmode_blocking)[1] for sensor in sensors]

    obstacles = ['Obj0', 'Obj1', 'Obj2', 'Obj3', 'Obj4', 'Obj5', 'Obj6', 'Obj7', 'Obj8', 'Obj9', 'Obj10']
    obst_handles = [vrep.simxGetObjectHandle(clientID, obj, vrep.simx_opmode_blocking)[1] for obj in obstacles]

    top_view = vrep.simxGetObjectHandle(clientID, 'top_view', vrep.simx_opmode_blocking)[1]

    # _, motor1 = vrep.simxGetObjectHandle(clientID,'motor1',vrep.simx_opmode_blocking)


    beta = b / n
    alpha = 2 * a * sin(beta / 2)
    gamma = [-c / n, -c / n, -c / n, -c / n, -c / n, -c / n, -c / n, -c / n]
    omega = -2

    # --------------------- Start the simulation

    # start our simulation in lockstep with our code
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)


    for motor_handle in motor_handles:
        _ = vrep.simxSetJointForce(clientID, motor_handle, 5, vrep.simx_opmode_blocking)
        if _ != 0: raise Exception()


    for sensor_handle in sensor_handles:
        _, _, _, _, _ = vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)


    obst_count = r.randint(5,8)
    for i in range(obst_count):
        obst = r.choice(obst_handles)
        _, [obst_copy] = vrep.simxCopyPasteObjects(clientID, [obst], vrep.simx_opmode_blocking)
        x = round(r.uniform(-5, 8), 3)
        y = round(r.uniform(-7.5, 8), 3)
        _ = vrep.simxSetObjectPosition(clientID, obst_copy, -1, (x, y, 0.25), vrep.simx_opmode_oneshot)
        _ = vrep.simxSetObjectOrientation(clientID, obst_copy, obst_copy, (r.randint(0, 180), 0, 0),
                                          vrep.simx_opmode_oneshot)

    vrep.simxSynchronousTrigger(clientID)

    _, res, img = vrep.simxGetVisionSensorImage(clientID, top_view, 0, vrep.simx_opmode_blocking)
    sleep(3)

    img = np.array(img,dtype = np.uint8)
    img.resize([res[0],res[1],3])
    img = cv2.flip(img,0)

    cv2.imshow("img",img)
    cv2.waitKey()

    paths = rPoly(img)

    t = threading.Thread(target=update_direction)
    t.daemon = True
    t.start()

    count = 0.0

    while count < 50:

        i = 1
        gamma[0] = -c / n
        if change:
            print("new c: ", c)
            change = False
            t = threading.Thread(target=update_direction)
            t.daemon = True
            t.start()

        for motor_handle in motor_handles:

            phi = alpha * sin(omega * count + (i - 1) * beta) + gamma[i - 1]

            _ = vrep.simxSetJointTargetPosition(clientID, motor_handle, phi, vrep.simx_opmode_blocking)
            if _ != 0:
                print(_)
                raise Exception()
            sleep(0.001)
            i += 1

        new_sensor_reading = []
        for sensor_handle in sensor_handles:
            error, state, point, _, _ = vrep.simxReadProximitySensor(clientID, sensor_handle,
                                                                     vrep.simx_opmode_streaming)
            if not state:
                new_sensor_reading.append(11.)
            else:
                reading = sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2)
                new_sensor_reading.append(reading)

        # print (count, new_sensor_reading )

        i = 7
        while i > 0:
            gamma[i] = gamma[i - 1]
            i -= 1

        vrep.simxSynchronousTrigger(clientID)
        count += dt
        count = round(count, 1)

    # -----------------------Stop the simulation
    _ = vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
    if _ != 0: raise Exception()
    t.join()
    vrep.simxFinish(clientID)

else:
    print('Failed connecting to remote API server')

print('\nProgram ended')
cv2.destroyAllWindows()
