import numpy as np

def controlMoment(feedback,qc,Kp,Kd):

	q = feedback[0:4]
	w = feedback[4:7]
	
	qErrFull = np.matmul(B_qErr(qc), q)
	
	if qErrFull[3] < 0:
		qErrFull *= -1
	
	
	qErr = qErrFull[:3]
	
	
	
	M = -Kp*qErr - Kd*w
	
	
	return M,qErrFull
	
	
def B_qErr(qc):
	B = np.zeros((4,4))
	B[0,0] =  qc[3]
	B[0,1] =  qc[2]
	B[0,2] = -qc[1]
	B[0,3] = -qc[0]
	B[1,0] = -qc[2]
	B[1,1] =  qc[3]
	B[1,2] =  qc[0]
	B[1,3] = -qc[1]
	B[2,0] =  qc[1]
	B[2,1] = -qc[0]
	B[2,2] =  qc[3]
	B[2,3] = -qc[2]
	B[3,0] =  qc[0]
	B[3,1] =  qc[1]
	B[3,2] =  qc[2]
	B[3,3] =  qc[3]
	return B	
