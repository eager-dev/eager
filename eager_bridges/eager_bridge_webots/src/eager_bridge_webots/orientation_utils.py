import math


def quad_to_axis_angle(quad):
    angle = 2 * math.acos(quad[3])
    denom = math.sqrt(1 - quad[3] * quad[3])
    if denom == 0:
        return 1, 0, 0, 0
    x = quad[0] / denom
    y = quad[1] / denom
    z = quad[2] / denom
    return x, y, z, angle


def quaternion_multiply(Q0, Q1):
    x0 = Q0[0]
    y0 = Q0[1]
    z0 = Q0[2]
    w0 = Q0[3]

    x1 = Q1[0]
    y1 = Q1[1]
    z1 = Q1[2]
    w1 = Q1[3]

    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    return [Q0Q1_x, Q0Q1_y, Q0Q1_z, Q0Q1_w]
