import numpy as np


def get_data():
    tx, ty, tz = map(float, input().split())
    qx, qy, qz, qw = map(float, input().split())
    x, y, z = map(float, input().split())
    return (tx, ty, tz), (qx, qy, qz, qw), (x, y, z)


def transform(v, q, p):
    t = np.array([v[0], v[1], v[2]])
    p_map = np.array([p[0], p[1], p[2]])
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]
    R = np.array([
    [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
    [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
    [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]])
    p_aruco = R.T @ (p_map - t)
    print(round(p_aruco[0], 3), round(p_aruco[1], 3), round(p_aruco[2], 3))


def main():
    v, q, p = get_data()
    transform(v, q, p)


if __name__ == '__main__':
    main()
