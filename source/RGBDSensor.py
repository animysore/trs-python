import numpy as np

class RGBDSensor:
  def __init__(self, vrep, vrep_id, rbh_id):
    self.vrep_id = vrep_id
    res, self.xyzSensor = vrep.simxGetObjectHandle(vrep_id, 'xyzSensor'+rbh_id, vrep.simx_opmode_oneshot_wait)
    res, self.rgbSensor = vrep.simxGetObjectHandle(vrep_id, 'rgbSensor'+rbh_id, vrep.simx_opmode_oneshot_wait)
    res, self.rgbdCasing = vrep.simxGetObjectHandle(vrep_id, 'rgbdSensor'+rbh_id, vrep.simx_opmode_oneshot_wait)
    
  def scan(self, vrep, opmode) -> np.ndarray:
    """
      Read from xyz sensor.

      (C) Copyright Renaud Detry 2013.
      Distributed under the GNU General Public License.
      (See http://www.gnu.org/copyleft/gpl.html)
    """

    # Again, the data comes in a funny format. Use the lines below to move the data to a matrix
    res, det, (auxData, auxPacketInfo) = vrep.simxReadVisionSensor(self.vrep_id, self.xyzSensor, opmode)
    # vrchk(vrep, res, true);
    width = int(auxPacketInfo[0])
    height = int(auxPacketInfo[1])
    
    # Each column of pts has [x;y;z;distancetosensor]
    pts = np.reshape(auxPacketInfo[2:], (4, width*height), order='F')
    pts = pts[:, pts[3,:]<4.9999]
    
    return pts