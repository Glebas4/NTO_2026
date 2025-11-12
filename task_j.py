import numpy as np
from scipy.spatial.transform import Rotation as R


def get_data():
    tx, ty, tz = map(float, input().split())
    qx, qy, qz, qw = map(float, input().split())
    x, y, z = map(float, input().split())
    return (tx, ty, tz), (qx, qy, qz, qw), (x, y, z)

def transform(v, q, p):
    t = np.array([t[0], t[1], t[2]])
    p_map = np.array([p[0], p[1], p[2]])
    rotation = R.from_quat([q[0], q[1], q[2], q[3]])
    point = rotation.inv().apply(p_map - t)
    print(round(point[0]), round(point[1]), round(point[2]))


def main():
    v, q, p = get_data()
    transform(v, q, p)


if __name__ == '__main__':
    main()
