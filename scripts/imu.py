from plt_utils import drawer
from helper import get_array_fields
from helper import get_files_in_dir
from helper import sort_param_files
import matplotlib.pyplot as plt
import numpy as np

params_iter_dir = '/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/params_iter'
figure_save_dir = '/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/figures'

figure_params_colors = ['r', 'g', 'b', 'k']
figure_color_alpha = 1.0
figure_marker_size = 3


def get_sensor_fields(fields, is_time=False):
    param_map = {}
    for file in files:
        params = get_array_fields(file, fields)
        for key in params:
            param_map.setdefault(key, []).append(params[key])
    if is_time:
        return param_map

    for k1 in param_map:
        vec_ls = []
        for elem in param_map[k1]:
            vec = []
            for k2 in elem:
                vec.append(elem[k2])
            vec_ls.append(vec)
        param_map[k1] = vec_ls
    return param_map


def draw_trans(axs, data):
    elems = [[], [], []]

    for i in range(len(data)):
        for j in [0, 1, 2]:
            elems[j].append(data[i][j])

    label = [drawer.math_symbols('{^{b^c}p_{b^i}}(x)'),
             drawer.math_symbols('{^{b^c}p_{b^i}}(y)'),
             drawer.math_symbols('{^{b^c}p_{b^i}}(z)')]

    for i in range(len(elems)):
        axs[i].plot(
            np.array(elems[i]), c=figure_params_colors[i], label=label[i],
            alpha=figure_color_alpha, marker='o', ms=figure_marker_size
        )
        min = np.min(elems[i])
        max = np.max(elems[i])
        span = max - min
        drawer.set_yticks(axs[i], min - span * 0.1, max + span * 0.1, 2)
        drawer.set_label_decimal(axs[i], '%.3f')
        axs[i].legend()
        drawer.add_grids(axs[i])


def draw_quat(axs, data):
    elems = [[], [], [], []]

    for i in range(len(data)):
        for j in [0, 1, 2, 3]:
            elems[j].append(data[i][j])

    label = [drawer.math_symbols(r'{^{b^c}_{b^i}{q}(x)}'),
             drawer.math_symbols(r'{^{b^c}_{b^i}{q}(y)}'),
             drawer.math_symbols(r'{^{b^c}_{b^i}{q}(z)}'),
             drawer.math_symbols(r'{^{b^c}_{b^i}{q}(w)}')]

    for i in range(len(elems)):
        axs[i].plot(np.array(elems[i]), c=figure_params_colors[i], label=label[i],
                    alpha=figure_color_alpha, marker='o', ms=figure_marker_size)
        min = np.min(elems[i])
        max = np.max(elems[i])
        span = max - min
        drawer.set_yticks(axs[i], min - span * 0.1, max + span * 0.1, 2)
        drawer.set_label_decimal(axs[i], '%.3f')
        axs[i].legend()
        drawer.add_grids(axs[i])


def draw_time(ax, data):
    ax.plot(np.array(data), c=figure_params_colors[0], label=drawer.math_symbols('{^{b^c}t_{b^i}}'),
            alpha=figure_color_alpha, marker='o', ms=figure_marker_size)
    min = np.min(data)
    max = np.max(data)
    span = max - min
    drawer.set_yticks(ax, min - span * 0.1, max + span * 0.1, 2)
    drawer.set_label_decimal(ax, '%.3f')
    ax.legend()
    drawer.add_grids(ax)


if __name__ == '__main__':
    files = sort_param_files(get_files_in_dir(params_iter_dir, '.json'))
    trans_dict = get_sensor_fields(['CalibParam', 'EXTRI', 'POS_BiInBc'])
    quat_dict = get_sensor_fields(['CalibParam', 'EXTRI', 'SO3_BiToBc'])
    time_dict = get_sensor_fields(['CalibParam', 'TEMPORAL', 'TIME_OFFSET_BiToBc'], True)

    num = len(trans_dict)

    for key in trans_dict:
        drawer.set_fig_size(16.0, 5.0)
        fig, axs = plt.subplots(3, 3, sharex=True)
        draw_trans([axs[0, 0], axs[1, 0], axs[2, 0]], trans_dict[key])
        draw_quat([axs[0, 1], axs[1, 1], axs[2, 1], axs[0, 2]], quat_dict[key])
        draw_time(axs[1, 2], time_dict[key])

        drawer.erase_ax(axs[2, 2])

        fig.suptitle('Topic: \'' + key + '\'')

        drawer.show_figure(figure_save_dir + '/imu' + key.replace("/", "_") + '.png')
