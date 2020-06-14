import numpy as np
from robopy import transl, trotx, troty, trotz

from .utils import homtrans

class Hokuyo:
  def __init__(self, vrep, vrep_id, rbh_id):
    """
      Initialize Hokuyo sensor in VREP
      This function starts the Hokuyo sensor, and it computes the transformations
      between the Hokuyo frame and the youBot reference frame h.ref.
      These transformations are stored in h.hokuyo1Trans and h.hokuyo2Trans
        (C) Copyright Renaud Detry 2013.
      Distributed under the GNU General Public License.
      (See http://www.gnu.org/copyleft/gpl.html)
    """
    self.vrep_id = vrep_id
    
    # The Hokuyo sensor is implemented with two planar sensors that each cover 120 degrees:
    res, self.hokuyo1 = vrep.simxGetObjectHandle(vrep_id, 'fastHokuyo_sensor1'+rbh_id, vrep.simx_opmode_oneshot_wait)
    res, self.hokuyo2 = vrep.simxGetObjectHandle(vrep_id, 'fastHokuyo_sensor2'+rbh_id, vrep.simx_opmode_oneshot_wait)
    
    res, self.ref = vrep.simxGetObjectHandle(vrep_id, 'youBot_center'+rbh_id, vrep.simx_opmode_oneshot_wait)
    
    # Turn the Hokuyo on (constantly)
    res = vrep.simxSetIntegerSignal(vrep_id, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot)
    # Display the red laser beams of active sensors
    res=  vrep.simxSetIntegerSignal(vrep_id, 'displaylasers', 1, vrep.simx_opmode_oneshot)

    # In order to build a map, you will need to transfer the data coming from the
    # Hokuyo to the world reference frame. The transformation between the frame of
    # hokuyoXHandle and the world is not accessible. However, you can access
    # the transformation between h.hokuyoX and h.ref (right below), and you
    # can access the transformation between h.ref and the world (see below, search for
    # "Stream wheel angles and robot pose"). By combining these two transformations,
    # you can transform the Hokuyo data to the world frame and build your map.
    # Do not forget that you can use the functions from the robot toolbox to compute
    # transformations, see page 23 of the book, e.g. the functions se2(), inv(),
    # h2e(), e2h(), homtrans(), ...
    res, self.hokuyo1Pos = vrep.simxGetObjectPosition(vrep_id, self.hokuyo1, self.ref, vrep.simx_opmode_oneshot_wait)
    res, self.hokuyo1Euler = vrep.simxGetObjectOrientation(vrep_id, self.hokuyo1, self.ref, vrep.simx_opmode_oneshot_wait)
    res, self.hokuyo2Pos = vrep.simxGetObjectPosition(vrep_id, self.hokuyo2, self.ref, vrep.simx_opmode_oneshot_wait)
    res, self.hokuyo2Euler = vrep.simxGetObjectOrientation(vrep_id, self.hokuyo2, self.ref, vrep.simx_opmode_oneshot_wait)

    # Compute the transformations between the two Hokuyo subsensors and the youBot
    # ref frame h.ref
    self.hokuyo1Trans = np.linalg.multi_dot([transl(self.hokuyo1Pos), trotx(self.hokuyo1Euler[0]), troty(self.hokuyo1Euler[1]), trotz(self.hokuyo1Euler[2])])
    self.hokuyo2Trans = np.linalg.multi_dot([transl(self.hokuyo2Pos), trotx(self.hokuyo2Euler[0]), troty(self.hokuyo2Euler[1]), trotz(self.hokuyo2Euler[2])])
  
  def scan(self, vrep, opmode, trans=None) -> (np.ndarray, np.ndarray):
    """
    Reads from Hokuyo sensor.

    (C) Copyright Renaud Detry 2013.
    Distributed under the GNU General Public License.
    (See http://www.gnu.org/copyleft/gpl.html)
    """
    pts1 = []
    pts2 = []
    obst1 = []
    obst2 = []

    if trans is not None:
      t1 = np.dot(trans, self.hokuyo1Trans)  
      t2 = np.dot(trans, self.hokuyo2Trans)  
    else:
      t1 = self.hokuyo1Trans
      t2 = self.hokuyo2Trans

    # The Hokuyo data comes in a funny format. Use the code below to move it
    # to a Matlab matrix
    res, det, (auxData, auxPacketInfo) = vrep.simxReadVisionSensor(self.vrep_id, self.hokuyo1, opmode)
    if res == 0:
      width = int(auxPacketInfo[0])
      pts1 = np.reshape(auxPacketInfo[2:], (4, width), order='F')
      # Each column of pts1 has [x;y;z;distancetosensor]
      # The Hokuyo sensor has a range of 5m. If there are no obstacles, a point
      # is returned at the 5m limit. As we do not want these points, we throw
      # away all points that are 5m far from the sensor.
      obst1 = pts1[3,:]<4.9999
      pts1 = pts1[:3,:]

    # Process the other 120 degrees      
    res, det, (auxData, auxPacketInfo) = vrep.simxReadVisionSensor(self.vrep_id, self.hokuyo2, opmode)
    if res == 0:
      width = int(auxPacketInfo[0])
      pts2 = np.reshape(auxPacketInfo[2:], (4, width), order='F')
      obst2 = pts2[3,:]<4.9999
      pts2 = pts2[:3,:]

    scanned_points = np.hstack((homtrans(t1, pts1), homtrans(t2, pts2)))
    contacts = np.hstack((obst1,obst2))
    
    return scanned_points, contacts    