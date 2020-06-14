import os 
import time
import math
import atexit
import configparser
import numpy as np
from matplotlib import path
import matplotlib.pyplot as plt

# Read config and load VREP paths  
cfg = configparser.ConfigParser()
cfg.read('config.ini')

os.environ['VREP'] = cfg['Simulator']['VREP']
os.environ['VREP_LIBRARY'] = cfg['Simulator']['VREP_LIBRARY']

# Import VREP api and TRS classes 
from api import vrep
from trs.Youbot import Youbot
from trs.utils import angdiff, cleanup_vrep

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

  # Minimum and maximum angles for all joints. Only useful to implement custom IK. 
  armJointRanges = [[-2.9496064186096, 2.9496064186096],
                    [-1.5707963705063, 1.308996796608],
                    [-2.2863812446594, 2.2863812446594],
                    [-1.7802357673645, 1.7802357673645],
                    [-1.5707963705063, 1.5707963705063]]

  # Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
  startingJoints = [0, 30.91 * math.pi / 180, 52.42 * math.pi / 180, 72.68 * math.pi / 180, 0]
  
  # Preset values for the demo. 
  print('Starting robot')
    
  # Define the preset pickup pose for this demo. 
  pickupJoints = [90 * math.pi / 180, 19.6 * math.pi / 180, 113 * math.pi / 180, - 41 * math.pi / 180, 0]

  # Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
  # They are adapted at each iteration by the code. 
  forwBackVel = 0 # Move straight ahead. 
  rightVel = 0 # Go sideways. 
  rotateRightVel = 0 # Rotate. 
  prevOrientation = 0 # Previous angle to goal (easy way to have a condition on the robot's angular speed). 
  prevPosition = 0 # Previous distance to goal (easy way to have a condition on the robot's speed).

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
  
  # Initialise the plot. 
  plotData = False
  if plotData:
    # Prepare the plot area to receive three plots: what the Hokuyo sees at the top (2D map), the point cloud and 
    # the image of what is in front of the robot at the bottom. 
    ax = plt.subplot(211)
    
    # Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display the area the robot can
    # see, by selecting the points within this mesh that are within the visibility range. 
    X, Y =  np.meshgrid(np.linspace(-5, 5, 40), np.linspace(-5.5, 2.5, 32)) # Values selected for the area the robot will explore for this demo. 
    X = np.reshape(X, (-1,), order='F') # Make a vector of the matrix X. 
    Y = np.reshape(Y, (-1,), order='F')
  # Make sure everything is settled before we start. 
  time.sleep(2)

  # Retrieve the position of the gripper. 
  res, homeGripperPosition = vrep.simxGetObjectPosition(id, h.Arm.ptip, h.Arm.armRef, vrep.simx_opmode_buffer)
  # vrchk(vrep, res, true)

  # Initialise the state machine.
  fsm = 'rotate'

  ## Start the demo. 
  while True:
    t = time.time() # See end of loop to see why it's useful. 
    
    if vrep.simxGetConnectionId(id) == -1:
      raise ConnectionAbortedError('Lost connection to remote API.')
 
    # Get the position and the orientation of the robot. 
    res, youbotPos = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer)
    #vrchk(vrep, res, true)
    res, youbotEuler = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer)
    #vrchk(vrep, res, true)

    ## Plot something if required. 
    if plotData:
      # Read data from the depth sensor, more often called the Hokuyo (if you want to be more precise about 
      # the way you control the sensor, see later for the details about this line or the file 
      # focused/youbot_3dpointcloud.m).
      # This function returns the set of points the Hokuyo saw in pts. contacts indicates, for each point, if it
      # corresponds to an obstacle (the ray the Hokuyo sent was interrupted by an obstacle, and was not allowed to
      # go to infinity without being stopped).
      pts, contacts = h.LaserScanner.scan(vrep, vrep.simx_opmode_buffer)

      # Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo (it returns the area that
      # is visible, but the visualisation draws a series of points that are within this visible area). 
      points = np.hstack((h.LaserScanner.hokuyo1Pos, pts[:], h.LaserScanner.hokuyo2Pos))
      polygon = path.Path(np.squeeze(np.asarray(points[0,:])), np.squeeze(np.asarray(points[1,:])))
            
      in_mask = polygon.contains_points(X, Y)

      # Plot those points. Green dots: the visible area for the Hokuyo. Red starts: the obstacles. Red lines: the
      # visibility range from the Hokuyo sensor. 
      # The youBot is indicated with two dots: the blue one corresponds to the rear, the red one to the Hokuyo
      # sensor position. 
      plt.subplot(211)
      plt.plot(X[in_mask], Y[in_mask], '.g')
      plt.plot(pts[0, contacts], pts[1, contacts], '*r')
      plt.plot(np.squeeze(np.asarray(points[0,:])), np.squeeze(np.asarray(points[1,:])), 'r')
      plt.plot(0, 0, 'ob')
      plt.plot(h.LaserScanner.hokuyo1Pos[0], h.hokuyo1Pos[1], 'or')
      plt.plot(h.LaserScanner.hokuyo2Pos[0], h.hokuyo2Pos[1], 'or')
      plt.set_xlim([-5.5, 5.5]) 
      plt.set_ylim([-5.5, 2.5]) 

    angl = -math.pi/2;

    ## Apply the state machine. 

    if fsm == 'rotate':
      # First, rotate the robot to go to one table.             
      # The rotation velocity depends on the difference between the current angle and the target. 
      rotateRightVel = angl - youbotEuler[2]
      
      # When the rotation is done (with a sufficiently high precision), move on to the next state. 
      if ((abs(angl - youbotEuler[2]) < .1 / 180 * math.pi) and
            (abs(prevOrientation - youbotEuler[2]) < .01 / 180 * math.pi)):
        rotateRightVel = 0;
        fsm = 'drive';
      
      prevOrientation = youbotEuler[2];

    elif fsm == 'drive':
      # Then, make it move straight ahead until it reaches the table (x = 3.167 m). 
      # The further the robot, the faster it drives. (Only check for the first dimension.)
      # For the project, you should not use a predefined value, but rather compute it from your map. 
      # Here, the goal is to reach y = -4.5. 
      forwBackVel = - 2 * (youbotPos[0] + 3.167)
      
      # If the robot is sufficiently close and its speed is sufficiently low, stop it and move its arm to 
      # a specific location before moving on to the next state.
      if (youbotPos[0] + 3.167 < .001) and (abs(youbotPos[0] - prevPosition) < .001):
        forwBackVel = 0
        
        # Change the orientation of the camera to focus on the table (preparation for next state). 
        vrep.simxSetObjectOrientation(id, h.RGBDSensor.rgbdCasing, h.ref, [0, 0, math.pi/4], vrep.simx_opmode_oneshot)
        
        # Move the arm to the preset pose pickupJoints (only useful for this demo; you should compute it based
        # on the object to grasp). 
        for i in range(5):
          res = vrep.simxSetJointTargetPosition(id, h.Arm.armJoints[i], pickupJoints[i], vrep.simx_opmode_oneshot)
          #vrchk(vrep, res, true)
        fsm = 'snapshot'

      prevPosition = youbotPos[0]
    
    elif fsm == 'snapshot':
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

      # Here, we only keep points within 1 meter, to focus on the table. 
      pts = pts[:3, pts[3, :] < 1]

      if plotData:
        ax = plt.subplot(223, projection='3d')
        ax.plot(pts[0, :], pts[2, :], pts[1, :], '*')
        #ax. view([-169 -46])

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
      if plotData:
        plt.imshow(image)

      # Next state.
      fsm = 'extend'
    
    elif fsm == 'extend':
      # Move the arm to face the object.
      # Get the arm position. 
      res, tpos = vrep.simxGetObjectPosition(id, h.Arm.ptip, h.Arm.armRef, vrep.simx_opmode_buffer)
      #vrchk(vrep, res, true)
      
      # If the arm has reached the wanted position, move on to the next state. 
      # Once again, your code should compute this based on the object to grasp. 
      if np.linalg.norm(np.array(tpos) - np.array([0.3259, -0.0010, 0.2951])) < .002:
      # Set the inverse kinematics (IK) mode to position AND orientation (km_mode = 2).
        res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait)
        #vrchk(vrep, res, true);
        fsm = 'reachout'
    
    elif fsm == 'reachout':
      # Move the gripper tip along a line so that it faces the object with the right angle.
      # Get the arm tip position. The arm is driven only by the position of the tip, not by the angles of 
      # the joints, except if IK is disabled.
      # Following the line ensures the arm attacks the object with the right angle. 
      res, tpos = vrep.simxGetObjectPosition(id, h.Arm.ptip, h.Arm.armRef, vrep.simx_opmode_buffer)
      #vrchk(vrep, res, true);

      # If the tip is at the right position, go on to the next state. Again, this value should be computed based
      # on the object to grasp and on the robot's position. 
      if tpos[0] > .39:
        fsm = 'grasp';

      # Move the tip to the next position along the line. 
      tpos[0] = tpos[0] + .01
      res = vrep.simxSetObjectPosition(id, h.Arm.ptarget, h.Arm.armRef, tpos, vrep.simx_opmode_oneshot)
      #vrchk(vrep, res, true)

    elif fsm == 'grasp':
      # Grasp the object by closing the gripper on it.
      # Close the gripper. Please pay attention that it is not possible to adjust the force to apply:  
      # the object will sometimes slip from the gripper!
      res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait)
      #vrchk(vrep, res);
      
      # Make MATLAB wait for the gripper to be closed. This value was determined by experiments. 
      time.sleep(2)

      # Disable IK; this is used at the next state to move the joints manually.
      res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait)
      #vrchk(vrep, res)
      fsm = 'backoff'
    
    elif fsm == 'backoff':

      for i in range(5):
        res = vrep.simxSetJointTargetPosition(id, h.Arm.armJoints[i], startingJoints[i], vrep.simx_opmode_oneshot)
        #vrchk(vrep, res, true)

        # Get the gripper position and check whether it is at destination (the original position).
        res, tpos = vrep.simxGetObjectPosition(id, h.Arm.ptip, h.Arm.armRef, vrep.simx_opmode_buffer)
        #vrchk(vrep, res, true)
        if np.linalg.norm(np.array(tpos) - np.array(homeGripperPosition)) < .02:
          # Open the gripper when the arm is above its base. 
          res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
          #vrchk(vrep, res);
        
        if np.linalg.norm(np.array(tpos) - np.array(homeGripperPosition)) < .002:
          fsm = 'finished';
 
    elif fsm == 'finished':
      # Demo done: exit the function. 
      time.sleep(3)
      break

    else:
      raise ValueError('Unknown state: ' + fsm)

    # Update wheel velocities using the global values (whatever the state is).
    h.drive(vrep, forwBackVel, rightVel, rotateRightVel)

    # Make sure that we do not go faster than the physics simulation (it is useless to go faster).
    elapsed = time.time() - t
    timeleft = timestep - elapsed
    if (timeleft > 0):
      time.sleep(min(timeleft, .01))
  
if __name__ == '__main__':
  youbot()

            
