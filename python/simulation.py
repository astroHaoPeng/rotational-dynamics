import numpy as np
import h5py
import os

def setupTime(tStart,tEnd,dt):
	
	N = int((tEnd-tStart)/dt+1)
	time,dt = np.linspace(tStart,tEnd,N,retstep=True)
	
	return time,dt,N

def limit(var,varLimit):
	# apply varLimit to each index of var 
	# where the magnitude of var exceeds the limit
	
	kLimit = abs(var) > varLimit
	var[kLimit] = np.sign(var[kLimit]) * varLimit
	return var

# 4th Order Runge Kutta Calculation
def RK4(f,x,u,dt):
    # Inputs: x[k], u[k], dt (time step, seconds)
    # Returns: x[k+1]
    
    # Calculate slope estimates
    K1 = f(x, u)
    K2 = f(x + K1 * dt / 2, u)
    K3 = f(x + K2 * dt / 2, u)
    K4 = f(x + K3 * dt, u)
    
    # Calculate x[k+1] estimate using combination of slope estimates
    x_next = x + 1/6 * (K1 + 2*K2 + 2*K3 + K4) * dt
    
    return x_next,K1
	
def saveData(filename,myData):
	# get full path for filename
	savePath = os.path.dirname(os.path.realpath(__file__))
	filepath = f'{savePath}\{filename}'
	
	print("Saving to...")
	print(f"\t{filepath}")
	with h5py.File(filepath,'w') as myFile:
		dList = []
		for myKey in myData.keys():
			dList.append(myFile.create_dataset(myKey,data=myData[myKey]))
	print('Done')
		
	

