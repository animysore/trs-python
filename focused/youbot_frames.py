import os, sys
import time
import math
import atexit
import configparser
import numpy as np
import matplotlib.pyplot as plt
from robopy import transl, trotx, troty, trotz

# Python disallows relative imports from non-packages, so we add parent directory to Path
sys.path.insert(1, os.path.join(sys.path[0], '..')) 

# Read config and load VREP paths 
cfg = configparser.ConfigParser()
cfg.read('../config.ini')

os.environ['VREP'] = cfg['Simulator']['VREP']
os.environ['VREP_LIBRARY'] = cfg['Simulator']['VREP_LIBRARY']

# Import VREP api and TRS classes 
from api import vrep
from trs.Youbot import Youbot
from trs.utils import cleanup_vrep, homtrans

"""
  Illustrates the V-REP MATLAB bindings, more specifically the way to take a 3D point cloud.
  
  (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017.
  Distributed under the GNU General Public License.
  (See http://www.gnu.org/copyleft/gpl.html)
"""
  
def youbot():
  print('Program started')

  ## Initiate the connection to the simulator. 
  vrep.simxFinish(-1)
  id = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

  if id < 0:
    print('Failed connecting to remote API server. Exiting.')
    return

  print('Connection {} to remote API server open '.format(id))

  # Make sure we close the connection whenever the script is interrupted.
  atexit.register(cleanup_vrep, vrep=vrep, id=id)

  # This will only work in "continuous remote API server service". 
  # See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
  vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait)

  # Retrieve all handles, mostly the Hokuyo.
  h = Youbot(vrep, id)
  ls = h.LaserScanner

  # Make sure everything is settled before we start (wait for the simulation to start). 
  time.sleep(.2)

  if vrep.simxGetConnectionId(id) == -1:
    raise ConnectionAbortedError('Lost connection to remote API.')
      
  # Read data from the depth camera (Hokuyo)
  # Get the position and orientation of the youBot in the world reference frame (as if with a GPS).
  res, youbotPos = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer)
  #vrchk(vrep, res, true); % Check the return value from the previous V-REP call (res) and exit in case of error.
  res, youbotEuler = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer)
  #vrchk(vrep, res, true);
  
  # Determine the position of the Hokuyo with global coordinates (world reference frame).                      
  trf = np.linalg.multi_dot((transl(youbotPos) , trotx(youbotEuler[0]) , troty(youbotEuler[1]) , trotz(youbotEuler[2])))
  worldHokuyo1 = homtrans(trf, np.array([[ls.hokuyo1Pos[0]], [ls.hokuyo1Pos[1]], [ls.hokuyo1Pos[2]]]))
  worldHokuyo2 = homtrans(trf, np.array([[ls.hokuyo2Pos[0]], [ls.hokuyo2Pos[1]], [ls.hokuyo2Pos[2]]]))
    
  # Use the sensor to detect the visible points, within the world frame. 
  pts, contacts = ls.scan(vrep, opmode=vrep.simx_opmode_buffer, trans=trf)
  world_points = np.hstack((worldHokuyo1, pts[:], worldHokuyo2))
      
  # Plot this data: delimit the visible area and highlight contact points.
  fig, ax = plt.subplots()
  ax.plot(pts[0, contacts], pts[1, contacts], '*r')
  ax.plot(np.squeeze(np.asarray(world_points[0,:])), np.squeeze(np.asarray(world_points[1,:])))
  ax.set_xlim([-10, 10])
  ax.set_ylim([-10, 10])
  plt.show()
  
if __name__ == '__main__':
  youbot()
