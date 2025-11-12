import rospy
import tf
from geometry_msgs.msg import PointStamped
import tf.transformations as tft
import numpy as np


def get_data():
    tx, ty, tz = map(float, input().split())
    qx, qy, qz, qw = map(float, input().split())
    x, y, z = map(float, input().split())
    return (tx, ty, tz), (qx, qy, qz, qw), (x, y, z)

def transform(trans, rot, point):
    R = tft.quaternion_matrix(rot)  
    R[0:3, 3] = trans 
    R_inv = np.linalg.inv(R)
    p_homog = np.array([xm, ym, zm, 1.0])
    p_aruco_homog = R_inv @ p_homog
    p_aruco = p_aruco_homog[:3]
    print(f"{p_aruco[0]:.3f} {p_aruco[1]:.3f} {p_aruco[2]:.3f}")


def main():
    rospy.init_node('task')
    v, q, p = get_data()
    transform(v, q, p)


if __name__ == '__main__':
    main()
