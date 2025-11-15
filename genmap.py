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
        self.name = name
        self.path = path
        self.pose = Pose()
        self.pose.position = Point(self.x, self.y, 0)
    
    def spawn(self):
        with open(self.path, 'r') as f:
            sdf_file = f.read()

        gen = spawn_service(model_name=self.name,
                             model_xml=sdf_file,
                             robot_namespace='',
                             initial_pose=self.pose,
                             reference_frame="world")
        print(gen.status_message, self.name, f"x={self.x}; y={self.y}")

    def delete(self):
        resp = delete_service(self.name)
        print(resp.status_message, self.name)


def gen_points():
    global points
    points = []
    while len(points) < 5:
        x = random.choice([1, -1])
        y = round(random.uniform(1.05, 8.95))
        if all(abs(point[1] - y) >= 0.75 for point in points): 
            points.append((x, y))

    return points


def main():
    points = gen_points()

    pipe_main = pipe(1, 1, path+"_main/pipe_main.sdf", "pipe_main")
    ps1 = pipe(points[0][0], points[0][1], path+"_small/pipe_small.sdf", "pipe_small_1")
    ps2 = pipe(points[1][0], points[1][1], path+"_small/pipe_small.sdf", "pipe_small_2")
    ps3 = pipe(points[2][0], points[2][1], path+"_small/pipe_small.sdf", "pipe_small_3")
    ps4 = pipe(points[3][0], points[3][1], path+"_small/pipe_small.sdf", "pipe_small_4")
    ps5 = pipe(points[4][0], points[4][1], path+"_small/pipe_small.sdf", "pipe_small_5")


    if len(sys.argv)>1:
        pipe_main.delete()
        ps1.delete()
        ps2.delete()
        ps3.delete()
        ps4.delete()
        ps5.delete()
    else:
        pipe_main.spawn()
        ps1.spawn()
        ps2.spawn()
        ps3.spawn()
        ps4.spawn()
        ps5.spawn()


if __name__ == '__main__':
    main()
