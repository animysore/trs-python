import os, sys
import time
import math
import atexit
import configparser

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
  The aim of this code is to show small examples of controlling the arm of the robot in V-REP. 

  (C) Copyright Renaud Detry 2013, Mathieu Baijot 2017.
  Distributed under the GNU General Public License.
  (See http://www.gnu.org/copyleft/gpl.html)
"""
  
def youbot():
  print('Program started')

  # Initiate the connection to the simulator. 
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

  # Preset values for the demo. 

  # Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
  startingJoints = [-90, 30.91 * math.pi / 180, 52.42 * math.pi / 180, 72.68 * math.pi / 180, 0]
  
  # Set the arm to its starting configuration. Several orders are sent sequentially, but they must be followed 
  # by the simulator without being interrupted by other computations (hence the enclosing simxPauseCommunication 
  # calls). 
  res = vrep.simxPauseCommunication(id, True) # Send order to the simulator through vrep object. 
  #vrchk(vrep, res); # Check the return value from the previous V-REP call (res) and exit in case of error.
  
  for i in range(5):
    res = vrep.simxSetJointTargetPosition(id, h.Arm.armJoints[i], startingJoints[i], vrep.simx_opmode_oneshot)
    #vrchk(vrep, res, true);
  
  res = vrep.simxPauseCommunication(id, False) 
  #vrchk(vrep, res);
  
  # Initialise the state machine.
  fsm = 'noIK'

  ## Start the demo. 
  while True:
    t = time.time() # See end of loop to see why it's useful. 
    
    if vrep.simxGetConnectionId(id) == -1:
      raise ConnectionAbortedError('Lost connection to remote API.')

    ## Apply the state machine. 

    # First example: move the arm without using the IK (inverse kinematics) solver, by explicitly setting the value 
    # for each of the joints. 
    if fsm == 'noIK':
      # Define each angle of the robot arm.
      chooseAngle = [180 * math.pi / 180, - 25 * math.pi / 180, 75 * math.pi / 180, - 17 * math.pi / 180, 90 * math.pi / 180]

      # Apply the value to each articulation.
      for i in range(5):
        res = vrep.simxSetJointTargetPosition(id, h.Arm.armJoints[i], chooseAngle[i], vrep.simx_opmode_oneshot)
        # vrchk(vrep, res, true)

      # Wait until the robot arm is in position.
      time.sleep(5)
      fsm = 'useIK'
        
    # Second example: move the arm by using the IK solver, so that you only have to specify the position of 
    # the arm tip. With this, you can move the point between the two tongs of the gripper so that it coincides with
    # the object to grasp (the tip): the gripper will magically be at the right position. 
    elif fsm == 'useIK':
      # Get the arm tip position. 
      [res, tpos] = vrep.simxGetObjectPosition(id, h.Arm.ptip, h.Arm.armRef, vrep.simx_opmode_buffer)
      #vrchk(vrep, res, true)
      
      # Set the inverse kinematics (IK) mode to position AND orientation (km_mode = 2). The solver will decide the
      # values for the joint angles so that the arm tip is at the given position with the given orientation (given
      # with a later simulator call). 
      res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait)
      #vrchk(vrep, res, true);
      
      # Set the new position to the expected one for the gripper (predetermined value).
      tpos = [tpos[0] - 0.1, tpos[1] + 0.3, tpos[2] - 0.3]
      res = vrep.simxSetObjectPosition(id, h.Arm.ptarget, h.Arm.armRef, tpos, vrep.simx_opmode_oneshot)
      #vrchk(vrep, res, true);
      # You could do the same with the orientation using vrep.simxSetObjectOrientation. The details are in the
      # documentation: 
      # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm#simxSetObjectOrientation
      
      # Wait long enough so that the tip is at the right position and go on to the next state. 
      time.sleep(5)
      fsm = 'rotGrip'
  
    # Rotate the tip. It can be actuated as any joint of the arm. 
    elif fsm == 'rotGrip':
      # Remove the inverse kinematics (IK) mode so that joint angles can be set individually (it was reenabled by
      # the previous state). 
      res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait)
      #vrchk(vrep, res, true);
      
      # Set the new gripper angle to 0°.
      res = vrep.simxSetJointTargetPosition(id, h.Arm.armJoints[4], 0, vrep.simx_opmode_oneshot)
      #vrchk(vrep, res, true);
      
      # Make MATLAB wait long enough so that the tip is at the right position and go on to the next state. 
      # This value was determined by experiments. 
      time.sleep(5)
      fsm = 'grasp'
        
    # Close the gripper. It is not possible to determine the force to apply and the object will sometimes slip 
    # from the gripper!
    elif fsm == 'grasp':
      res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait)
      #vrchk(vrep, res);
      
      # Make MATLAB wait for the gripper to be closed. This value was determined by experiments. 
      time.sleep(3)
      fsm = 'release'
    
    # Open the gripper.
    elif fsm == 'release':
      res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait)
      #vrchk(vrep, res);
      
      # Make MATLAB wait for the gripper to be opened. This value was determined by experiments. 
      time.sleep(5)
      fsm = 'finished'

    # Demo done: exit the function. 
    elif fsm == 'finished':
      time.sleep(3)
      break

    else:
      raise ValueError('Unknown state: ' + fsm)

    # Make sure that we do not go faster than the physics simulation (it is useless to go faster).
    elapsed = time.time() - t
    timeleft = timestep - elapsed
    if (timeleft > 0):
      time.sleep(min(timeleft, .01))
  
if __name__ == '__main__':
  youbot()

            
