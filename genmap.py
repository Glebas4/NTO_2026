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


def gen_points(x1, x2, y1, y2, n):
    points = []
    while len(points) != n:
        x = round(random.uniform(x1, x2), 2)
        y = round(random.uniform(y1, y2), 2)
        side = random.choice([True, False])
        #if side:
         #   x+=0.71
          #  y+=0.71
        if all(math.sqrt((point[0] - x)**2 + (point[1] - y)**2) >=0.75 for point in points): #Если гипотенуза соединяющая точки >=1 то координата довабляется
            points.append((x, y))

    return points


def gen_pipes(l): #l - длина трубы
    #Чтобы угол меж основной трубой и врезкой был <= 30,то диапазон угла между ними будет равен разнице смещения и 30 градусов
    #main_angle = random.randint(0, 20)
    main_angle = 30 # 30 degrees = 0.6 Yaw Gazebo
    rot_angle = random.randint(main_angle-30, main_angle+30) 

    rad = math.radians(main_angle)
    x = round(l * math.sin(rad), 4) + 1  #Смещение второй точки по X и Y 
    y = round(l * math.cos(rad), 4) + 0.95 #По идее 1,но лучше 0.95

    loc_angle_rot_pipe = math.radians(rot_angle-main_angle)
    xk = round(l * math.cos(rad), 4) + 0.95
    yk = round(l * math.sin(rad), 4) + 0.95 # 0.95 тк радиус трубы 5 см и нужно чтобы она не выпирала
    #Начало и угол осн трубы; Начало 2ой трубы(конец 1ой) и угол; конец 2 трубы
    return (1, 1, -main_angle/50), (x, y, -rot_angle/50), (xk, yk)


def main():
    if len(sys.argv)>1:
        pmain =  pipe(name="pipe_main")
        prot  =  pipe(name="pipe_rot")
        #ps1   =  pipe(name="pipe_small_1")
        #ps2   =  pipe(name="pipe_small_2")
        #ps3   =  pipe(name="pipe_small_3")
        #ps4   =  pipe(name="pipe_small_4")
        #ps5   =  pipe(name="pipe_small_5")
    
        pmain.delete()
        prot.delete()
        #ps1.delete()
        #ps2.delete()
        #ps3.delete()
        #ps4.delete()
        #ps5.delete()
    else:
        pmain_cords, prot_cords, prot_end = gen_pipes(l=3)
        #pnt_pipe_main = gen_points(x1=1, x2=prot_cords[0], y1=1, y2=prot_cords[1], n=3)
        #pnt_pipe_rot = gen_points(x1=prot_cords[0], x2=prot_end[0], y1=prot_cords[1], y2=prot_end[1], n=2)

    #pipe_main = pipe(pmain_cords[0], pmain_cords[1], path+"_main/pipe_main.sdf", "pipe_main", pmain_cords[3])
    #pipe_rot = pipe(prot_cords[0], prot_cords[1], path+"_main/pipe_main.sdf", "pipe_rot", pmain_cords[3])

    #ps1 - pipe_small №1
        pmain = pipe(pmain_cords[0], pmain_cords[1], path+"_main/pipe_main.sdf", "pipe_main", pmain_cords[2])
        prot = pipe(prot_cords[0], prot_cords[1], path+"_main/pipe_main.sdf", "pipe_rot", prot_cords[2])
        #ps3 = pipe(pnt_pipe_main[0][0], pnt_pipe_main[0][1], path+"_small/pipe_small.sdf", "pipe_small_1", pmain_cords[2]-1.8)
        #ps4 = pipe(pnt_pipe_main[1][0], pnt_pipe_main[1][1], path+"_small/pipe_small.sdf", "pipe_small_2", pmain_cords[2]-1.8)
        #ps5 = pipe(pnt_pipe_rot[0][0], pnt_pipe_rot[0][1], path+"_small/pipe_small.sdf", "pipe_small_3", prot_cords[2]-1.8)
        #ps6 = pipe(pnt_pipe_rot[1][0], pnt_pipe_rot[1][1], path+"_small/pipe_small.sdf", "pipe_small_4", prot_cords[2]-1.8)

        pmain.spawn()
        prot.spawn()
        #ps3.spawn()
        #ps4.spawn()
        #ps5.spawn()
        #ps6.spawn()
        #pipe_main.spawn()
        #pipe_rot.spawn()


if __name__ == '__main__':
    main()
