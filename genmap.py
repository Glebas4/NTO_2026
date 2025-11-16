import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tft
import random
import sys
import math


rospy.init_node("genmap")


spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
path = "/home/clover/catkin_ws/src/sitl_gazebo/models/pipe"


class pipe:
    def __init__(self, x, y, path, name, angle=0):
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


def gen_points():
    global points
    points = []
    while len(points) < 5:
        x = random.choice([1, 3])
        y = round(random.uniform(1.05, 8.95), 2)
        if all(abs(point[1] - y) >= 0.75 for point in points): 
            points.append((x, y))

    return points


def gen_pipes(l): #l - длина трубы
    #Чтобы угол меж основной трубой и врезкой был <= 30,то диапазон угла между ними будет равен разнице смещения и 30 градусов
    main_angle = random.randint(0, 20)
    rot_angle = random.randint(180-main_angle-30, 180-main_angle+30) 

    main_angle = math.radians(main_angle)
    x = l * math.sin(main_angle) - 1  #Смещение второй точки по X и Y 
    y = l * math.cos(main_angle) - 1
    
    return (1, 1, main_angle), (x, y, rot_angle)


def main():
    points = gen_points()
    pmain_cords, prot_cords = gen_pipes(l=1)

    #pipe_main = pipe(pmain_cords[0], pmain_cords[1], path+"_main/pipe_main.sdf", "pipe_main", pmain_cords[3])
    #pipe_rot = pipe(prot_cords[0], prot_cords[1], path+"_main/pipe_main.sdf", "pipe_rot", pmain_cords[3])

    #ps1 - pipe_small №1
    ps1 = pipe(pmain_cords[0], pmain_cords[1], path+"_small/pipe_small.sdf", "pipe_main", pmain_cords[2])
    ps2 = pipe(prot_cords[0], prot_cords[1], path+"_small/pipe_small.sdf", "pipe_rot", prot_cords[2])
    #ps3 = pipe(points[2][0], points[2][1], path+"_small/pipe_small.sdf", "pipe_small_3")
    #ps4 = pipe(points[3][0], points[3][1], path+"_small/pipe_small.sdf", "pipe_small_4")
    #ps5 = pipe(points[4][0], points[4][1], path+"_small/pipe_small.sdf", "pipe_small_5")


    if len(sys.argv)>1:
        #pipe_main.delete()
        #pipe_rot.delete()
        ps1.delete()
        ps2.delete()
        #ps3.delete()
        #ps4.delete()
        #ps5.delete()
    else:
        ps1.spawn()
        ps2.spawn()
        #pipe_main.spawn()
        #pipe_rot.spawn()


if __name__ == '__main__':
    main()
