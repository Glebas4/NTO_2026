import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point
import random
import sys
import math

spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
path = "/home/clover/catkin_ws/src/sitl_gazebo/models/pipe"


class pipe:
    def __init__(self, x, y, path, name):
        self.x = x
        self.y = y
        self.z = 0
        self.name = name
        self.path = path
        self.pose = Pose()
        self.pose.position = Point(self.x, self.y, self.z)
    

    def spawn(self):
        with open(self.path, 'r') as f:
            sdf_file = f.read()

        gen = spawn_service(model_name=self.name,
                             model_xml=sdf_file,
                             robot_namespace='',
                             initial_pose=self.pose,
                             reference_frame="world")
        print(gen.status_message, self.name, "pipe")

    def delete(self):
        resp = delete_service(self.name)
        print(resp.status_message, self.name, self.color)


def main():
    pipe_main = pipe(1, 1, path+"_main/pipe_main.sdf", "pipe_main")
    if len(sys.argv)>1:
        pipe_main.delete()
    else:
        pipe_main.spawn()



if __name__ == '__main__':
    main()
