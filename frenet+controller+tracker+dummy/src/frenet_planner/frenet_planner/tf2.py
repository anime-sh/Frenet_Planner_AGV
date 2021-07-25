import numpy as np
import math

# convert euler angles to quaternion
# math referenced here -> https://stackoverflow.com/questions/53033620/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
def quaternion_from_euler(yaw, pitch, roll):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

# convert quaternion to euler angles
# math referenced here -> https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def euler_from_quaternion(qx,qy,qz,qw):
    sinr_cosp = 2*(qw*qx + qy*qz)
    cosr_csop = 1 - 2*(qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp,cosr_csop)

    sinp = 2*(qw*qy-qz*qx)

    pitch = 0.0

    if(abs(sinp)>=1):
        pitch = math.copysign(math.pi/2 , sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2*(qw*qx + qx*qy)
    cosy_cosp = 1 - 2*(qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp,cosy_cosp)

    return [roll,pitch,yaw]