import vrep 
from math import sin
from math import pi
from math import sqrt
from time import sleep
 
# close any open connections
vrep.simxFinish(-1)
# Connect to the V-REP continuous server
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5) 
 
if clientID != -1: # if we connected successfully
	print ('Connected to remote API server')

	# --------------------- Setup the simulation 
	vrep.simxSynchronous(clientID,True)
	# move simulation ahead one time step
	#vrep.simxSynchronousTrigger(clientID)

	dt = .1
	vrep.simxSetFloatingParameter(clientID,	vrep.sim_floatparam_simulation_time_step,dt,vrep.simx_opmode_oneshot)

	n =8
	a =pi/3
	b =1.5*pi
	c =2.0

	motors =['motor1','motor2','motor3','motor4','motor5','motor6','motor7']
	motor_handles = [vrep.simxGetObjectHandle(clientID,name, vrep.simx_opmode_blocking)[1] for name in motors]

	sensors = ['sensor1r','sensor1l','sensor2r','sensor2l','sensor3r','sensor3l','sensor4r','sensor4l',	'sensor5r','sensor5l','sensor6r','sensor6l','sensor7r','sensor7l','sensor8r','sensor8l']
	sensor_handles = [vrep.simxGetObjectHandle(clientID, sensor, vrep.simx_opmode_blocking)[1] for sensor in sensors]

	#_, motor1 = vrep.simxGetObjectHandle(clientID,'motor1',vrep.simx_opmode_blocking)
	beta = b/n
	alpha = 2*a*sin(beta/2)
	gamma = -c/n
	omega = -1
	
	# --------------------- Start the simulation
	 
	# start our simulation in lockstep with our code
	vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)

	for motor_handle in motor_handles:
		_=vrep.simxSetJointForce(clientID,motor_handle,5,vrep.simx_opmode_blocking)
		if _ !=0 : raise Exception()

	for sensor_handle in sensor_handles:
		_,_,_,_,_ = vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)

	count=0.
	
	while count<50:

		i=1

		for motor_handle in motor_handles:

			phi = alpha*sin(omega*count + (i-1)*beta) + gamma

			_=vrep.simxSetJointTargetPosition(clientID,motor_handle,phi,vrep.simx_opmode_blocking)
			if _ !=0 : 
				print (_)
				raise Exception()
			sleep(0.001)
			i+=1

		new_sensor_reading = []
		for sensor_handle in sensor_handles:
			error, state, point, _, _ = vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)
			if not(state):
				new_sensor_reading.append(21.)
			else:
				reading = sqrt(point[0]**2 + point[1]**2 + point[2]**2)
				new_sensor_reading.append(reading)

		print (count, new_sensor_reading )


		vrep.simxSynchronousTrigger(clientID)
		count += dt
		count = round(count,1)


	# -----------------------Stop the simulation
	_=vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
	if _ !=0 : raise Exception()

	vrep.simxFinish(clientID)

else:
	print ('Failed connecting to remote API server')

print ('Program ended')