import numpy as np
import time
import math
from .Hokuyo import Hokuyo
from .RGBDSensor import RGBDSensor
from .YoubotArm import YoubotArm

class Youbot:
  def __init__(self, vrep, vrep_id, robot_id=-1):
    """
      Initialize youBot
    
      (C) Copyright Renaud Detry 2013.
      Distributed under the GNU General Public License.
      (See http://www.gnu.org/copyleft/gpl.html)
    
      Retrieve all handles, and stream arm and wheel joints, the robot's pose,
      the Hokuyo, and the arm tip pose.
    """
    self.vrep_id = vrep_id

    if robot_id >= 0:
      rbh_id = '#' + str(robot_id)
    else:
      rbh_id = ''

    self.LaserScanner = Hokuyo(vrep, vrep_id, rbh_id)
    self.RGBDSensor = RGBDSensor(vrep, vrep_id, rbh_id)
    self.Arm = YoubotArm(vrep, vrep_id, rbh_id)

    self.ref = self.LaserScanner.ref

    wheelJoints = [-1,-1,-1,-1] # front left, rear left, rear right, front right
    res, wheelJoints[0] = vrep.simxGetObjectHandle(vrep_id, 'rollingJoint_fl'+rbh_id, vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[1] = vrep.simxGetObjectHandle(vrep_id, 'rollingJoint_rl'+rbh_id, vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[2] = vrep.simxGetObjectHandle(vrep_id, 'rollingJoint_rr'+rbh_id, vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[3] = vrep.simxGetObjectHandle(vrep_id, 'rollingJoint_fr'+rbh_id, vrep.simx_opmode_oneshot_wait)

    for i in range(4):
      vrep.simxSetJointTargetVelocity(vrep_id, wheelJoints[i], 0, vrep.simx_opmode_oneshot)
    
    self.wheelJoints = wheelJoints
    self.previousForwBackVel = 0
    self.previousLeftRightVel = 0
    self.previousRotVel = 0    
    
    self.test_connection(vrep)
  
  def test_connection(self, vrep):
    """
      Stream wheel angles, Hokuyo data, and robot pose (see usage below)
      Wheel angles are not used in this example, but they may/will be necessary in
      your project.
    """
    vrep_id = self.vrep_id
    for i in range(4):
      res = vrep.simxGetJointPosition(vrep_id, self.wheelJoints[i], vrep.simx_opmode_streaming)

    res = vrep.simxGetObjectPosition(vrep_id, self.ref, -1, vrep.simx_opmode_streaming)
    res = vrep.simxGetObjectOrientation(vrep_id, self.ref, -1, vrep.simx_opmode_streaming)
    res = vrep.simxReadVisionSensor(vrep_id, self.LaserScanner.hokuyo1, vrep.simx_opmode_streaming)
    res = vrep.simxReadVisionSensor(vrep_id, self.LaserScanner.hokuyo2, vrep.simx_opmode_streaming)
    
    # Stream the arm joint angles and the tip position/orientation
    res = vrep.simxGetObjectPosition(vrep_id, self.Arm.ptip, self.Arm.armRef, vrep.simx_opmode_streaming)
    res = vrep.simxGetObjectOrientation(vrep_id, self.Arm.otip, self.Arm.r22, vrep.simx_opmode_streaming)
    for i in range(4):
      res = vrep.simxGetJointPosition(vrep_id, self.Arm.armJoints[i], vrep.simx_opmode_streaming)
    
    vrep.simxGetPingTime(vrep_id) # make sure that all streaming data has reached the client at least once
    

  def drive(self, vrep, forwBackVel, leftRightVel, rotVel) -> None:
    """
      Sets the youBot wheel speed to achieve the given forward, lateral
      and rotational velocities. The velocities are normalized to say
      within the bounds of the actuator capabilities.
      (C) Copyright Renaud Detry 2013.
      Distributed under the GNU General Public License.
      (See http://www.gnu.org/copyleft/gpl.html)
    """
    pParam=20
    maxV=12
    pParamRot=10
    maxVRot=4
    accelF=0.05

    forwBackVel=forwBackVel*pParam
    leftRightVel=leftRightVel*pParam
    v=math.sqrt(forwBackVel*forwBackVel+leftRightVel*leftRightVel)
    if v>maxV:
      forwBackVel=forwBackVel*maxV/v
      leftRightVel=leftRightVel*maxV/v

    rotVel=rotVel*pParamRot
    if (abs(rotVel)>maxVRot):
      rotVel=maxVRot*rotVel/abs(rotVel)

    df=forwBackVel-self.previousForwBackVel
    ds=leftRightVel-self.previousLeftRightVel
    dr=rotVel-self.previousRotVel

    if (abs(df)>maxV*accelF): df=abs(df)*(maxV*accelF)/df

    if (abs(ds)>maxV*accelF): ds=abs(ds)*(maxV*accelF)/ds

    if (abs(dr)>maxVRot*accelF): dr=abs(dr)*(maxVRot*accelF)/dr

    forwBackVel=self.previousForwBackVel+df
    leftRightVel=self.previousLeftRightVel+ds
    rotVel=self.previousRotVel+dr
    self.previousForwBackVel=forwBackVel
    self.previousLeftRightVel=leftRightVel
    self.previousRotVel=rotVel

    # Update wheel velocities
    res = vrep.simxPauseCommunication(self.vrep_id, True)

    opmode = vrep.simx_opmode_oneshot
    res = vrep.simxSetJointTargetVelocity(self.vrep_id, self.wheelJoints[0], -forwBackVel-leftRightVel+rotVel, opmode)
    res = vrep.simxSetJointTargetVelocity(self.vrep_id, self.wheelJoints[1], -forwBackVel+leftRightVel+rotVel, opmode)
    res = vrep.simxSetJointTargetVelocity(self.vrep_id, self.wheelJoints[2], -forwBackVel-leftRightVel-rotVel, opmode)
    res = vrep.simxSetJointTargetVelocity(self.vrep_id, self.wheelJoints[3], -forwBackVel+leftRightVel-rotVel, opmode)
    
    res = vrep.simxPauseCommunication(self.vrep_id, False)
