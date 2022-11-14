import numpy as np


def constraint(value, lower, upper):
    if value > upper:
        value = upper
    if value < lower:
        value = lower
    return value


def compute_radius(point1, point2, point3):
    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3
    v1 = 2 * (x2 - x1)
    v2 = 2 * (y2 - y1)
    v3 = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1
    v4 = 2 * (x3 - x2)
    v5 = 2 * (y3 - y2)
    v6 = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2
    x = (v2 * v6 - v5 * v3) / (v2 * v4 - v5 * v1)
    y = (v4 * v3 - v1 * v6) / (v2 * v4 - v5 * v1)
    radius = np.sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1))
    return radius


def find_closest_point(point, path):
    x = np.array([point[:2]])
    dist = np.sqrt(np.linalg.norm(x - path, axis=1))
    index = np.argmin(dist)
    return index


def compute_error(pos, point):
    x0, y0, theta0 = pos
    x, y = point
    error = np.sin(theta0) * (x - x0) - np.cos(theta0) * (y - y0)
    return error
