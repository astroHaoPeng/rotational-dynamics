import numpy as np
from simulation import limit

def skew(x):
    xSkew = np.zeros((3,3))
    xSkew[0,1] = -x[2]
    xSkew[0,2] =  x[1]
    xSkew[1,0] =  x[2]
    xSkew[1,2] = -x[0]
    xSkew[2,0] = -x[1]
    xSkew[2,1] =  x[0]
    return xSkew
	
def B_qdot(w):
	B = np.zeros((4,4))
	B[0,1] =  w[2]
	B[0,2] = -w[1]
	B[0,3] =  w[0]
	B[1,1] = -w[2]
	B[1,2] =  w[0]
	B[1,3] =  w[1]
	B[2,0] =  w[1]
	B[2,1] = -w[0]
	B[2,3] =  w[2]
	B[3,0] = -w[0]
	B[3,1] = -w[1]
	B[3,2] = -w[2]
	return B
	

def rbodyCalc(X,U):
	"""
	Rigid body model
	
		Currently only used for rotational dynamics
	
	INPUTS:
		t = current time
		X = State vector
			Body Rates (rad/s)
				0  = wb1
				1  = wb2
				2  = wb3
			Inertia Tensor 
				3  = J00
				4  = J01
				5  = J02
				6  = J10
				7  = J11
				8  = J12
				9  = J20
				10 = J21
				11 = J22
			Attitude expressed as quaternion
				12 = q1
				13 = q2
				14 = q3
				15 = q4
		U = Input Vector
			External Moment
				0  = M1
				1  = M2
				2  = M3
	OUTPUTS:
		XDOT = dState vector
			Angular accleration
				0  = wdot1
				1  = wdot2
				2  = wdot3
			Inertia Tensor rate of change
				3  = Jdot00
				4  = Jdot01
				5  = Jdot02
				6  = Jdot10
				7  = Jdot11
				8  = Jdot12
				9  = Jdot20
				10 = Jdot21
				11 = Jdot22				
			Attitude Rates expressed as quaternion
				12 = qdot1
				13 = qdot2
				14 = qdot3
				15 = qdot4
	"""
	# State Vector
	wb = np.array([ X[0], X[1], X[2] ])
	J  = np.array([ [ X[3],  X[4],  X[5] ],
					[ X[6],  X[7],  X[8] ],
					[ X[9], X[10], X[11] ] ])
	q = np.array([ X[12], X[13], X[14], X[15] ])	
	
	# Input Vector
	M = np.array([ U[0], U[1], U[2] ])
	
	# Solve Euler’s rotational equation of motion in matrix form for wdot
	J_wdot = M - np.matmul( skew(wb), np.matmul(J,wb) )
	wdot = np.matmul(J.T, J_wdot)
	
	# solve for qdot
	qdot = 0.5 * np.matmul(B_qdot(wb), q)
	
	# Chane in Inertia Tensor
	Jdot = np.zeros_like(J)
	
	# Build XDOT Vector
	XDOT = np.zeros(16)
	XDOT[0:3] = wdot
	XDOT[3:6] = Jdot[0,:]
	XDOT[6:9] = Jdot[1,:]
	XDOT[9:12] = Jdot[2,:]
	XDOT[12:16] = qdot

	
	return XDOT
	
	