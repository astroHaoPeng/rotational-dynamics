
import matplotlib.pyplot as plt
import numpy as np
import h5py
import os

from simulation import saveData
import transform as trn

DEG2RAD = np.pi / 180.0
RAD2DEG = 1/DEG2RAD


def postCalcs(Y):

	Y['euler321'] = trn.EP2Euler321(Y['quat'])
	Y['omegaLateral'] = np.sqrt(Y['omega'][1]**2 + Y['omega'][2]**2)
		
	return Y
		
		
	
def generatePlots(filename):
	savePath = os.path.dirname(os.path.realpath(__file__))
	filepath = f'{savePath}\{filename}'
	
	with h5py.File(filepath,'r') as X:
		time = X['time'][:]
		omega = X['omega'][:]
		moment = X['moment'][:]
		quat = X['quat'][:]
		qErr = X['qErr'][:]
		euler321 = X['euler321'][:]
	

	fig,ax = plt.subplots(1,1, sharex=True)
	ax.plot(time[0,:], omega[0,:]*RAD2DEG,'r',label='w1')
	ax.plot(time[0,:], omega[1,:]*RAD2DEG,'g',label='w2')
	ax.plot(time[0,:], omega[2,:]*RAD2DEG,'b',label='w3')
	ax.legend()
	ax.grid()
	ax.set_ylabel('Omega (deg/s)')
	ax.set_xlabel('Time(s)')
	
	fig,ax = plt.subplots(1,1, sharex=True)
	ax.plot(time[0,:], moment[0,:],'r',label='Mx')
	ax.plot(time[0,:], moment[1,:],'g',label='My')
	ax.plot(time[0,:], moment[2,:],'b',label='Mz')
	ax.legend()
	ax.grid()
	ax.set_ylabel('Moment (N-m)')
	ax.set_xlabel('Time(s)')	

	fig,ax = plt.subplots(1,1, sharex=True)
	ax.plot(time[0,:], quat[0,:],'r',label='q1')
	ax.plot(time[0,:], quat[1,:],'g',label='q2')
	ax.plot(time[0,:], quat[2,:],'b',label='q3')
	ax.plot(time[0,:], quat[3,:],'k',label='q4')
	ax.legend()
	ax.grid()
	ax.set_ylabel('Quaternion')
	ax.set_xlabel('Time(s)')
	
	fig,ax = plt.subplots(1,1, sharex=True)
	ax.plot(time[0,:], qErr[0,:],'r',label='q1')
	ax.plot(time[0,:], qErr[1,:],'g',label='q2')
	ax.plot(time[0,:], qErr[2,:],'b',label='q3')
	ax.plot(time[0,:], qErr[3,:],'k',label='q4')
	ax.legend()
	ax.grid()
	ax.set_ylabel('qErr')
	ax.set_xlabel('Time(s)')

	fig,ax = plt.subplots(1,1, sharex=True)
	ax.plot(time[0,:], euler321[0,:]*RAD2DEG,'r',label='yaw')
	ax.plot(time[0,:], euler321[1,:]*RAD2DEG,'g',label='pitch')
	ax.plot(time[0,:], euler321[2,:]*RAD2DEG,'b',label='roll')
	ax.legend()
	ax.grid()
	ax.set_ylabel('euler321 (deg)')
	ax.set_xlabel('Time(s)')
	
	

	plt.show()