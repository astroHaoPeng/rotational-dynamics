import numpy as np

EPS = 7./3 - 4./3 -1


def euler3212DCM(ang):

	ang = euler321LimitRange(ang)

	ang1 = ang[0]
	ang2 = ang[1]
	ang3 = ang[2]

	c1 = np.cos(ang1)
	s1 = np.sin(ang1)
	c2 = np.cos(ang2)
	s2 = np.sin(ang2)
	c3 = np.cos(ang3)
	s3 = np.sin(ang3)

	dcm = np.zeros((3,3))
	dcm[0,0] = c2*c1
	dcm[0,1] = c2*s1
	dcm[0,2] = -s2
	dcm[1,0] = s3*s2*c1 - c3*s1
	dcm[1,1] = s3*s2*s1 + c3*c1
	dcm[1,2] = s3*c2
	dcm[2,0] = c3*s2*c1 + s3*s1
	dcm[2,1] = c3*s2*s1 - s3*c1
	dcm[2,2] = c3*c2

	return dcm

def euler321LimitRange(ang):
	# limit euler321 angles to 
	# yaw   +/- 180 deg
	# pitch +/- 180 deg
	# roll  +/- 180 deg	
	# input angles as rad

	
	for k in range(3):
		if ang[k] > np.pi:
			ang[k] += -2.0*np.pi
		elif ang[k] <= np.pi:
			ang[k] +=  2.0*np.pi
			
	return ang

	
def EP2Euler321(q):
	"""
	EP2Euler321

		E = EP2Euler321(Q) translates the Euler parameter vector
		Q into the corresponding (3-2-1) Euler angle set.
	"""

	r,c = q.shape

	# map to other convention
	q0 = q[3,:]
	q1 = q[0,:]
	q2 = q[1,:]
	q3 = q[2,:]


	euler321 = np.zeros((3,c))
	euler321[0,:] = np.arctan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)
	euler321[1,:] = np.arcsin(-2*(q1*q3-q0*q2))
	euler321[2,:] = np.arctan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3)
	return euler321	
	
	
def dcm2q(dcm,wantAngLast=False):
	"""
	Determine quaternion corresponding to dcm using
	the stanley method. 
	
	Flips sign to always return shortest path quaterion
	
	default to q0,q1,q2,q3 but
	set wantAngLast=True
	to output q1,q2,q3,q4 convention
	"""
	
	tr = np.trace(dcm)
	
	q = np.zeros(4)
	q[0] = 1+tr
	q[1] = 1+2*dcm[0,0]-tr
	q[2] = 1+2*dcm[1,1]-tr
	q[3] = 1+2*dcm[2,2]-tr
	q *= 0.25
	
	maxCase = np.argmax(q)
	qq = q
	
	if   maxCase == 0:
		# angle only component
		q[0] = np.sqrt(qq[0]);
		q[1] = 0.25*(dcm[1,2]-dcm[2,1])/q[0];
		q[2] = 0.25*(dcm[2,0]-dcm[0,2])/q[0];
		q[3] = 0.25*(dcm[0,1]-dcm[1,0])/q[0];
	elif maxCase == 1:
		q[1] = np.sqrt(qq[1]);
		q[0] = 0.25*(dcm[1,2]-dcm[2,1])/q[1];
		if q[0]<0:
			q[1] = -q[1];
			q[0] = -q[0];
		q[2] = 0.25*(dcm[0,1]+dcm[1,0])/q[1];
		q[3] = 0.25*(dcm[2,0]+dcm[0,2])/q[1];
	elif maxCase == 2:
		q[2] = np.sqrt(qq[2]);
		q[0] = 0.25*(dcm[2,0]-dcm[0,2])/q[2];
		if q[0]<0:
			q[2] = -q[2];
			q[0] = -q[0];
		q[1] = 0.25*(dcm[0,1]+dcm[1,0])/q[2];
		q[3] = 0.25*(dcm[1,2]+dcm[2,1])/q[2];
	elif maxCase == 3:
		q[3] = np.sqrt(qq[3]);
		q[0] = 0.25*(dcm[0,1]-dcm[1,0])/q[3];
		if q[0]<0:
			q[3] = -q[3];
			q[0] = -q[0];
		q[1] = 0.25*(dcm[2,0]+dcm[0,2])/q[3];
		q[2] = 0.25*(dcm[1,2]+dcm[2,1])/q[3];	
		
	# make sure quaternion is valid
	q = quatNorm(q)
		
			
	if wantAngLast:
		angPart = q[0]
		q[:-1] = q[1:]
		q[-1] = angPart
			
	return q
	
def quatNorm(q):
	normSquared = 0
	for each in q:
		normSquared += each*each
	if (abs(normSquared)-1) < (4*EPS):
		q /= np.sqrt(normSquared)
	
	return q


	
	
	

	
