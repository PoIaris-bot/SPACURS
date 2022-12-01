import numpy as np
import configparser


def get_area(file_name, safe_margin=1):
    config = configparser.ConfigParser()
    config.read(file_name)
    x0, y0 = eval(config.get('COORDINATE', 'x0')), eval(config.get('COORDINATE', 'y0'))
    x1, y1 = eval(config.get('COORDINATE', 'x1')), eval(config.get('COORDINATE', 'y1'))
    x2, y2 = eval(config.get('COORDINATE', 'x2')), eval(config.get('COORDINATE', 'y2'))
    x3, y3 = eval(config.get('COORDINATE', 'x3')), eval(config.get('COORDINATE', 'y3'))

    x_min = max(x0, x1)
    x_max = min(x2, x3)
    y_min = max(y0, y3) + safe_margin
    y_max = min(y1, y2) - safe_margin
    return x_min, y_min, x_max, y_max


def rrt(x0, x_goal, config_file_name):
    print('path planning start')
    x_min, y_min, x_max, y_max = get_area(config_file_name)
    x_min, y_min, x_max, y_max = (0, 0, 50, 40)
    if x_goal[0] < x_min:
        x_goal[0] = x_min
    if x_goal[0] > x_max:
        x_goal[0] = x_max
    if x_goal[1] < y_min:
        x_goal[1] = y_min
    if x_goal[1] > y_max:
        x_goal[1] = y_max

    step = 5

    x_root = x0[:2] + step * np.array([np.cos(x0[2]), np.sin(x0[2])])
    sample_rate = 0.3
    max_tree_size = 500

    tree = x_root.reshape(1, 2)
    parent = [-1]
    i = 0
    flag = False
    while i < max_tree_size:
        print(i)
        if np.random.rand() < sample_rate:
            x_rand = x_goal
        else:
            x_rand = np.array([
                x_min + np.random.rand() * (x_max - x_min),
                y_min + np.random.rand() * (y_max - y_min)
            ])
        dist = np.linalg.norm(x_rand - tree, axis=1)
        dist_near = np.min(dist)
        idx_near = np.argmin(dist)
        x_near = tree[idx_near, :]
        x_new = x_near + step / dist_near * (x_rand - x_near)

        if x_min < x_new[0] < x_max and y_min < x_new[1] < y_max:
            tree = np.concatenate((tree, x_new.reshape(1, 2)), axis=0)
            parent.append(idx_near)
            i += 1
            if np.linalg.norm(x_new - x_goal) < 2 * step:
                flag = True
                break
    if flag:
        path = np.concatenate((tree[-1, :].reshape(1, 2), x_goal.reshape(1, 2)))
        idx = parent[-1]
        while idx >= 0:
            path = np.concatenate((tree[idx, :].reshape(1, 2), path))
            idx = parent[idx]
        path = np.concatenate((x0[:2].reshape(1, 2), path))
        print('Path found')
        return path
    else:
        print('Path not found')
        return None
