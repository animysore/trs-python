import numpy as np
from numpy.matlib import repmat

def homtrans(T:np.ndarray, p: np.ndarray) -> np.ndarray:
  
  homP = np.vstack((p, np.ones((1, p.shape[1]))))
  homRes = np.dot(T, homP)

  eucRes = np.divide(homRes[:-1, :], repmat(homRes[-1,:], homRes.shape[0]-1, 1))

  return eucRes

def angdiff(source, target) -> float:
  a = target - source
  a = (a + 180) % 360 - 180
  return a

def cleanup_vrep(vrep, id):
  print('Closing connection :', id)
  vrep.simxStopSimulation(id, vrep.simx_opmode_oneshot_wait)
  vrep.simxFinish(id)
  print('Program ended')