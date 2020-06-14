import os 
import time
import math
import atexit

base_dir = '/home/aniruddha/Downloads/bin/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04/'
bindings = base_dir+'programming/remoteApiBindings/lib/lib/Ubuntu18_04/'

os.environ['VREP'] = base_dir
os.environ['VREP_LIBRARY'] = bindings

from vrep import vrep
from source.Youbot import Youbot
from source.utils import angdiff, cleanup_vrep

"""
  The aim of this code is to show small examples of controlling the displacement of the robot in V-REP. 
  
  (C) Copyright Renaud Detry 2013, Mathieu Baijot 2017.
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

  # The time step the simulator is using (your code should run close to it). 
  timestep = .05

  ## Preset values for the demo. 

  # Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
  # They are adapted at each iteration by the code. 
  forwBackVel = 0 # Move straight ahead. 
  rightVel = 0 # Go sideways. 
  rotateRightVel = 0 # Rotate. 

  # First state of state machine
  fsm = 'forward'
  print('Switching to state: ', fsm)

  ## Start the demo. 
  while True:
    t = time.time() # See end of loop to see why it's useful. 
    
    if vrep.simxGetConnectionId(id) == -1:
      raise ConnectionAbortedError('Lost connection to remote API.')
      

    # Get the position and the orientation of the robot. 
    res, youbotPos = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer)
    #vrchk(vrep, res, true) # Check the return value from the previous V-REP call (res) and exit in case of error.
    res, youbotEuler = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer)
    #vrchk(vrep, res, true)

    ## Apply the state machine. 
    if fsm == 'forward':
      # Make the robot drive with a constant speed (very simple controller, likely to overshoot). 
      # The speed is - 1 m/s, the sign indicating the direction to follow. Please note that the robot has
      # limitations and cannot reach an infinite speed. 
      forwBackVel = -1
      
      # Stop when the robot is close to y = - 6.5. The tolerance has been determined by experiments: if it is too
      # small, the condition will never be met (the robot position is updated every 50 ms); if it is too large,
      # then the robot is not close enough to the position (which may be a problem if it has to pick an object,
      # for example). 
      if abs(youbotPos[1] + 6.5) < .01:
        forwBackVel = 0 # Stop the robot. 
        fsm = 'backward'
        print('Switching to state: ', fsm)
        
    elif fsm == 'backward':
      # A speed which is a function of the distance to the destination can also be used. This is useful to avoid
      # overshooting: with this controller, the speed decreases when the robot approaches the goal. 
      # Here, the goal is to reach y = -4.5. 
      forwBackVel = - 2 * (youbotPos[1] + 4.5)
      #             ^^^   ^^^^^^^^^^^^^^^^^^^^
      #             |     distance to goal
      #             influences the maximum speed
      
      # Stop when the robot is close to y = 4.5. 
      if abs(youbotPos[1] + 4.5) < .01:
        forwBackVel = 0 # Stop the robot. 
        fsm = 'right'
        print('Switching to state: ', fsm)
        
    elif fsm == 'right':
      # Move sideways, again with a proportional controller (goal: x = - 4.5). 
      rightVel = - 2 * (youbotPos[0] + 4.5)
      
      # Stop at x = - 4.5
      if abs(youbotPos[0] + 4.5) < .01:
        rightVel = 0 # Stop the robot. 
        fsm = 'rotateRight'
        print('Switching to state: ', fsm)
        
    elif fsm == 'rotateRight':
      # Rotate until the robot has an angle of -pi/2 (measured with respect to the world's reference frame). 
      # Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
      # and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
      # the anguler speed becomes negative). 
      # youbotEuler(3) is the rotation around the vertical axis. 
      rotateRightVel = angdiff(- math.pi / 2, youbotEuler[2] ) # angdiff ensures the difference is between -pi and pi. 
      
      # Stop when the robot is at an angle close to -pi/2. 
      if abs(angdiff(- math.pi / 2, youbotEuler[2])) < .002:
        rotateRightVel = 0
        fsm = 'finished'
        print('Switching to state: ', fsm)
        
    elif fsm == 'finished':
      time.sleep(3)
      break
    else:
      raise ValueError('Unknown state: ' + fsm)

    # Update wheel velocities.
    h.drive(vrep, forwBackVel, rightVel, rotateRightVel)
    
    # What happens if you do not update the velocities? The simulator always considers the last speed you gave it,
    # until you set a new velocity. If you perform computations for several seconds in a row without updating the
    # speeds, the robot will continue to move --- even if it bumps into a wall. 

    # Make sure that we do not go faster than the physics simulation (it is useless to go faster).
    elapsed = time.time() - t
    timeleft = timestep - elapsed
    if (timeleft > 0):
      time.sleep(min(timeleft, .01))
  
if __name__ == '__main__':
  youbot()
