import os, sys
import time
import math
import atexit
import configparser
import numpy as np
import matplotlib.pyplot as plt

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

  # Retrieve all handles.
  h = Youbot(vrep, id)

  # Make sure everything is settled before we start (wait for the simulation to start). 
  time.sleep(.2)

  # Read data from the RGB camera. 
  # This starts the robot's camera to take a 2D picture of what the robot can see. 
  # Reading an image costs a lot to VREP (it has to simulate the image). It also requires a lot of bandwidth, 
  # and processing an image will take a long time in MATLAB. In general, you will only want to capture 
  # an image at specific times, for instance when you believe you're facing one of the tables or a basket.
  
  # Ask the sensor to turn itself on, take A SINGLE IMAGE, and turn itself off again. 
  # ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  # simxSetIntegerSignal          1        simx_opmode_oneshot_wait
  #         |
  #         handle_rgb_sensor
  res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait)
  #vrchk(vrep, res); # Check the return value from the previous V-REP call (res) and exit in case of error. 
  
  # Then retrieve the last picture the camera took. The image must be in RGB (not gray scale). 
  #      ^^^^^^^^^^^^^^^^^^^^^^^^^     ^^^^^^                            ^^^
  #      simxGetVisionSensorImage2     h.rgbSensor                       0
  # If you were to try to capture multiple images in a row, try other values than vrep.simx_opmode_oneshot_wait. 
  print('Capturing image...')
  res, resolution, image = vrep.simxGetVisionSensorImage(id, h.RGBDSensor.rgbSensor, 0, vrep.simx_opmode_oneshot_wait)
  #vrchk(vrep, res);
  width, height = resolution
  print('Captured {} pixels ({} x {}).'.format(width * height, width, height))

  image = np.reshape(image, (height, width, 3) )
  
  # Finally, show the image.
  plt.imshow(image)
  plt.show()
  
if __name__ == '__main__':
  youbot()
