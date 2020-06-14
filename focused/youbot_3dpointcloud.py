import os, sys
import time
import math
import atexit
import configparser
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Python disallows relative imports from non-packages, so we add parent directory to Path
sys.path.insert(1, os.path.join(sys.path[0], '..')) 

# Read Cond 
cfg = configparser.ConfigParser()
cfg.read('../config.ini')

os.environ['VREP'] = cfg['Simulator']['VREP']
os.environ['VREP_LIBRARY'] = cfg['Simulator']['VREP_LIBRARY']

# Import VREP api and TRS classes 
from api import vrep
from source.Youbot import Youbot
from source.utils import angdiff, cleanup_vrep

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

  # Make sure everything is settled before we start (wait for the simulation to start). 
  time.sleep(.2)

  if vrep.simxGetConnectionId(id) == -1:
    raise ConnectionAbortedError('Lost connection to remote API.')
      
  # Read data from the depth camera (Hokuyo)
  # Reading a 3D image costs a lot to VREP (it has to simulate the image). It also requires a lot of 
  # bandwidth, and processing a 3D point cloud (for instance, to find one of the boxes or cylinders that 
  # the robot has to grasp) will take a long time in MATLAB. In general, you will only want to capture a 3D 
  # image at specific times, for instance when you believe you're facing one of the tables.

  # Reduce the view angle to pi/8 in order to better see the objects. Do it only once. 
  # ^^^^^^     ^^^^^^^^^^    ^^^^                                     ^^^^^^^^^^^^^^^ 
  # simxSetFloatSignal                                                simx_opmode_oneshot_wait
  #            |
  #            rgbd_sensor_scan_angle
  # The depth camera has a limited number of rays that gather information. If this number is concentrated 
  # on a smaller angle, the resolution is better. pi/8 has been determined by experimentation.
  res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', math.pi / 8, vrep.simx_opmode_oneshot_wait)
  #vrchk(vrep, res); # Check the return value from the previous V-REP call (res) and exit in case of error.

  # Ask the sensor to turn itself on, take A SINGLE POINT CLOUD, and turn itself off again. 
  # ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  # simxSetIntegerSignal          1        simx_opmode_oneshot_wait
  #         |
  #         handle_xyz_sensor
  res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait)
  #vrchk(vrep, res);

  # Then retrieve the last point cloud the depth sensor took.
  # If you were to try to capture multiple images in a row, try other values than 
  # vrep.simx_opmode_oneshot_wait. 
  print('Capturing point cloud...')
  pts = h.RGBDSensor.scan(vrep, vrep.simx_opmode_oneshot_wait)
  # Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
  # the output data. To get a correct plot, you should invert the y and z dimensions. 

  # Plot all the points. 
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d') 
  ax.plot(pts[0, :], pts[2, :], pts[1, :], '*')
  
  # Plot the points of the wall (further away than 1.87 m, which is determined either in the simulator by measuring 
  # distances or by trial and error) in a different colour. This value is only valid for this robot position, of
  # course. This simple test ignores the variation of distance along the wall (distance between a point and several
  # points on a line). 
  ptsWall = pts[0:3, pts[3, :] >= 1.87]
  ax.plot(ptsWall[0, :], ptsWall[2, :], ptsWall[1, :], '.r')

  plt.show()
  
  # Also have a look to the function youbot_hokuyo, used in the main youbot function. It is simpler to use than what
  # this script showed, but with fewer functionalities. This function is also used in the next focused example,
  # youbot_frames.m. 
  
if __name__ == '__main__':
  youbot()
