import numpy as np
from simulation import limit

MS2S = 0.001

nominalThrust = 0.001
riseTime  = 10 * MS2S
decayTime = 10 * MS2S

tauRise  = riseTime  / 3
tauDecay = decayTime / 3

def firstOrderLag(x, xPrev, tau):
	# calculate xdot for linear first order equation with time constant (tau)
	# 95% rise/decay time = 3*tau
	return (x - xPrev) / tau
	


	

def rcsCalc(U):
	"""
	Reaction Control System model
	
		Currently really basic model
	
	INPUTS:
		t = current time
		X = State vector
			Thruster States
				0  = rollThruster
				1  = pitchThruster
				2  = yawThruster
		U = Input Vector
			Thruster Commands
				0  = rollThrusterCommand
				1  = pitchThrusterCommand
				2  = yawThrusterCommand
	OUTPUTS:
		XDOT = dState vector
			Thruster 1st order dState
				0  = rollThruster
				1  = pitchThruster
				2  = yawThruster
	"""

	
	# Input Vector
	thruster_command = np.array([ U[0], U[1], U[2] ])
	
	# State Vector
	#thruster_state = np.array([ X[0], X[1], X[2] ])
	thruster_state = np.zeros_like(thruster_command)	
	
	
	# Calculate thrust
	for k in range(3):
		thruster_state[k] = thruster_command[k]*nominalThrust
		
	thruster_state = limit(thruster_state,nominalThrust)



	
	return thruster_state
	
	