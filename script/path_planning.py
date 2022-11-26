import numpy as np


def rrt(x0, x_goal):
    x_min = 0
    x_max = 25
    y_min = 0
    y_max = 25
    step = 1.5

    x_root = x0[:2] + step * np.array([np.cos(x0[2]), np.sin(x0[2])])
    sample_rate = 0.3
    max_tree_size = 500

    tree = x_root.reshape(1, 2)
    parent = [-1]
    i = 0
    flag = False
    while i < max_tree_size:
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
