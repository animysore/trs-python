class YoubotArm:
  def __init__(self, vrep, vrep_id, rbh_id):
    # The project page ( http://renaud-detry.net/teaching/info0948/private/project.php )
    # contains information on the different control modes of the arm. Search for
    # km_mode on the project webpage to find the arm documentation. Read that documentation
    # before working with the code below.
    
    self.vrep_id = vrep_id

    res, self.armRef = vrep.simxGetObjectHandle(vrep_id, 'youBot_ref'+rbh_id, vrep.simx_opmode_oneshot_wait)
    
    # The *position* of this object always corresponds to the position of the tip of
    # the arm (the tip is somewhere between the two fingers)
    res, self.ptip = vrep.simxGetObjectHandle(vrep_id, 'youBot_gripperPositionTip'+rbh_id, vrep.simx_opmode_oneshot_wait)
    # In IK mode (km_mode set to 1 or 2), the robot will try to move the *position*
    # of ptip to the *position* of ptarget.
    res, self.ptarget = vrep.simxGetObjectHandle(vrep_id, 'youBot_gripperPositionTarget'+rbh_id, vrep.simx_opmode_oneshot_wait)
    # The *orientation* of this object always corresponds to the orientation of the tip of
    # the arm (the tip is somewhere between the two fingers)
    res, self.otip = vrep.simxGetObjectHandle(vrep_id, 'youBot_gripperOrientationTip'+rbh_id, vrep.simx_opmode_oneshot_wait)
    # In IK mode 2 (km_mode set to 2), the robot will try to move the *orienatation*
    # of otip to the *orientation* of otarget.
    res, self.otarget = vrep.simxGetObjectHandle(vrep_id, 'youBot_gripperOrientationTarget'+rbh_id, vrep.simx_opmode_oneshot_wait)
    # Tip orientations are easier to manipulate in the reference frame of Rectangle22,
    # because then the degree of freedom onto which the orientation controller acts
    # corresponds to one of the three Euler angles of the tip orientation.
    res, self.r22 = vrep.simxGetObjectHandle(vrep_id, 'Rectangle22'+rbh_id, vrep.simx_opmode_oneshot_wait)
    
    armJoints = [-1,-1,-1,-1,-1]
    
    for i in range(4):
      res, armJoints[i] = vrep.simxGetObjectHandle(vrep_id, f'youBotArmJoint{i}{rbh_id}', vrep.simx_opmode_oneshot_wait)
    
    self.armJoints = armJoints