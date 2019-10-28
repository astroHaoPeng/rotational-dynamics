import numpy as np
import h5py

from rigidBody import rbodyCalc
from rcsActuator import rcsCalc
from simulation import RK4,setupTime,saveData
from postProcessing import postCalcs,generatePlots
from control import controlMoment

import simulation as sim

import transform as trn

DEG2RAD = np.pi / 180.0
RAD2DEG = 1/DEG2RAD


# Simulation setup
filename = "rotationalDynamics.h5" 
tStart = 0
tEnd = 10
DT = 0.001
time,DT,N = setupTime(tStart,tEnd,DT)

# Rigid Body
# Inertia Tensor
J = np.zeros((3,3))
J[0,0] = 100.
J[1,1] = 100.
J[2,2] = 100.

#J[0,0] = 21400
#J[1,1] = 20100
#J[2,2] =  5000
#J[0,1] =  -2100
#J[0,2] =  -1800
#J[1,2] =   -500
#J[1,0] = J[0,1]
#J[2,0] = J[0,2]
#J[2,1] = J[1,2]

# Initial rates
w = np.zeros(3)
w[0] = 0 # deg/s
w[1] = 0 # deg/s
w[2] = 0 # deg/s
w *= DEG2RAD

# Initial Attitude
# euler321
euler321 = np.zeros(3)
euler321[0] = 0  # yaw (deg)
euler321[1] = 0  # pitch (deg)
euler321[2] = 0  # roll (deg)
euler321 *= DEG2RAD
# DCM
dcm = trn.euler3212DCM(euler321)
q = trn.dcm2q(dcm,wantAngLast=True)

# Build rbody state vector
X_rbody = np.zeros(16)
X_rbody[0:3] = w
X_rbody[3:6] = J[0,:]
X_rbody[6:9] = J[1,:]
X_rbody[9:12] = J[2,:]
X_rbody[12:16] = q

# Rigid Body Moment 
M = np.zeros(3)

feedback = np.zeros(7)


########### Run Sim
rbody={}
rbody['time']  = np.zeros((1,N))
rbody['omega'] = np.zeros((3,N))
rbody['quat']  = np.zeros((4,N))
rbody['moment']= np.zeros((3,N))
rbody['omegaDot']= np.zeros((3,N))
rbody['Ixx']= np.zeros((1,N))
rbody['Ixy']= np.zeros((1,N))
rbody['Iyz']= np.zeros((1,N))
rbody['qErr'] = np.zeros((4,N))


print("Running sim")


# Calculate command attitude
euler321_command = np.zeros(3)
euler321_command[0] = 50 # yaw (deg)
euler321_command[1] = 25 # pitch (deg)
euler321_command[2] = -5# roll (deg)
euler321_command *= DEG2RAD

rollRate = 0 # deg/s
for k,tk in enumerate(time):
	
	
	############# Outerloop attitude commands
	if tk > 1.0:
		# Calculate command attitude	
		euler321_command[0] += 0 # yaw (deg)
		euler321_command[1] += 0 # pitch (deg)	
		euler321_command[2] += rollRate *DEG2RAD * DT
		euler321_command = trn.euler321LimitRange(euler321_command)
		
	# DCM
	dcm_command = trn.euler3212DCM(euler321_command)
	q_command = trn.dcm2q(dcm_command,wantAngLast=True)
	
	############ Innerloop attitude control
	# Control for rbody_moment
	feedback[0:4] = X_rbody[12:16] # quat attitude
	feedback[4:7] = X_rbody[0:3] # omega
	Kp = np.array([1,1,1])
	Kd = np.array([1,1,1])
	M,qErr = controlMoment(feedback,q_command,Kp,Kd)
	
	########### Plant Model state space

	# rbody XDOT
	X_rbody_update,XDOT_rbody = RK4(rbodyCalc, X_rbody, M, DT)
	X_rbody_update[12:16] = trn.quatNorm(X_rbody_update[12:16])
	
	# Store current X,XDOT before updating state vector
	
	rbody['time'][:,k]     = tk
	rbody['omega'][:,k]    = X_rbody[0:3] 
	rbody['quat'][:,k]     = X_rbody[12:16]
	rbody['moment'][:,k]   = M
	rbody['omegaDot'][:,k] = XDOT_rbody[0:3]
	rbody['Ixx'][:,k]      = X_rbody[3]
	rbody['Ixy'][:,k]      = X_rbody[7]
	rbody['Iyz'][:,k]      = X_rbody[11]
	rbody['qErr'][:,k]     = qErr
	
	# Update for next iteration
	X_rbody = X_rbody_update
	
print("Sim complete")
print("Beginning post processing...")
rbody = postCalcs(rbody)
saveData(filename,rbody)

generatePlots(filename)


	
	
	


	
	
	
	














