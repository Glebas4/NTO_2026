import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tft
import random
import sys
import math
import numpy as np


rospy.init_node("genmap")


spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
path = "/home/clover/catkin_ws/src/sitl_gazebo/models/pipe"

points = []
main_pipe_angle = 0
main_pipe_start_x = 1
main_pipe_start_y = 1
main_pipe_end_x = 0
main_pipe_end_y = 0

rot_pipe_angle = 0
rot_pipe_end_x = 0
rot_pipe_end_y = 0



class pipe:
    def __init__(self, x=0, y=0, path=0, name=0, angle=0):
        self.x = x
        self.y = y

        self.name = name
        self.path = path

        self.pose = Pose()
        self.pose.position = Point(self.x, self.y, 0)

        self.angle = angle
        self.q = tft.quaternion_from_euler(0, 0, self.angle)
        self.pose.orientation = Quaternion(*self.q)
    
    def spawn(self):
        with open(self.path, 'r') as f:
            sdf_file = f.read()

        gen = spawn_service(model_name=self.name,
                             model_xml=sdf_file,
                             robot_namespace='',
                             initial_pose=self.pose,
                             reference_frame="world")
        print(gen.status_message, self.name, f"x={self.x}; y={self.y}; yaw={self.angle}")

    def delete(self):
        resp = delete_service(self.name)
        print(resp.status_message, self.name)


def gen_points(x1, x2, y1, y2, n, angle):
    global points
    pnts = []
    p1 = np.array([x1, y1])
    p2 = np.array([x2, y2])
    vector = p2 - p1
    while len(points) != n:
        t = random.uniform(0, 1)
        point = p1 + t * vector
        if all(np.linalg.norm(point - pnt) >=0.75 for pnt in points): #Расстояние между точками
            pnts.append(point)
            points.append(point)
    
    return pnts


def gen_pipes(l): #l - длина трубы
    #Чтобы угол меж основной трубой и врезкой был <= 30,то диапазон угла между ними будет равен разнице смещения и 30 градусов
    global rot_pipe_angle, main_pipe_angle, main_pipe_end_x, main_pipe_end_y, rot_pipe_end_x, rot_pipe_end_y
    main_pipe_angle = random.randint(0, 20)
    #main_angle = 30 # 30 degrees = 0.6 Yaw Gazebo
    rot_pipe_angle = random.randint(main_pipe_angle-30, main_pipe_angle+30) 
    rad = math.radians(main_pipe_angle)
    main_pipe_end_x = l * math.sin(rad) + 1  #Смещение второй точки по X и Y 
    main_pipe_end_y = l * math.cos(rad) + 1

    rad2 = math.radians(rot_pipe_angle)
    rot_pipe_end_x = l * math.sin(rad2) + main_pipe_angle
    rot_pipe_end_y = l * math.cos(rad2) + main_pipe_end_y



def main():
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10)
    if len(sys.argv)>1:
        pmain =  pipe(name="pipe_main")
        prot  =  pipe(name="pipe_rot")
        ps1   =  pipe(name="pipe_small_1")
        ps2   =  pipe(name="pipe_small_2")
        ps3   =  pipe(name="pipe_small_3")
        ps4   =  pipe(name="pipe_small_4")
        ps5   =  pipe(name="pipe_small_5")
    
        pmain.delete()
        prot.delete()
        ps1.delete()
        ps2.delete()
        ps3.delete()
        ps4.delete()
        ps5.delete()
    else:
        gen_pipes(l=3)
        main_pipe_vrezki = gen_points(x1=main_pipe_start_x, x2=main_pipe_end_x, y1=main_pipe_start_y, y2=main_pipe_end_y, n=3, angle=main_pipe_angle)
        rot_pipe_vrezki  = gen_points(x1=main_pipe_end_x, x2=rot_pipe_end_x, y1=main_pipe_end_y, y2=rot_pipe_end_y, n=2, angle=rot_pipe_angle)

        pmain = pipe(main_pipe_start_x, main_pipe_start_y, path+"_main/pipe_main.sdf", "pipe_main", main_pipe_angle)
        prot  = pipe(main_pipe_end_x, main_pipe_end_y, path+"_main/pipe_main.sdf", "pipe_rot", rot_pipe_angle)
        ps1   = pipe(main_pipe_vrezki[0][0], main_pipe_vrezki[0][1], path+"_small/pipe_small.sdf", "pipe_small_1", main_pipe_angle-1.8) #1.8 Yaw Gazebo = 90 градусов
        ps2   = pipe(main_pipe_vrezki[1][0], main_pipe_vrezki[1][1], path+"_small/pipe_small.sdf", "pipe_small_2", main_pipe_angle-1.8)
        ps3   = pipe(main_pipe_vrezki[2][0], main_pipe_vrezki[2][1], path+"_small/pipe_small.sdf", "pipe_small_3", main_pipe_angle-1.8)
        ps4   = pipe(rot_pipe_vrezki[0][0], rot_pipe_vrezki[0][1], path+"_small/pipe_small.sdf", "pipe_small_4", rot_pipe_angle-1.8)
        ps5   = pipe(rot_pipe_vrezki[1][0], rot_pipe_vrezki[1][1], path+"_small/pipe_small.sdf", "pipe_small_5", rot_pipe_angle-1.8)

        pmain.spawn()
        prot.spawn()
        ps1.spawn()
        ps2.spawn()
        ps3.spawn()
        ps4.spawn()
        ps4.spawn()


if __name__ == '__main__':
    main()
