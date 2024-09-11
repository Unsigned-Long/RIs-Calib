#  RIs-Calib: An Open-Source Spatiotemporal Calibrator for Multiple 3D Radars and IMUs Based on Continuous-Time Estimation
#  Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
#  https://github.com/Unsigned-Long/RIs-Calib.git
#
#  Author: Shuolong Chen (shlchen@whu.edu.cn)
#  GitHub: https://github.com/Unsigned-Long
#   ORCID: 0000-0002-5283-9057
#
#  Purpose: See .h/.hpp file.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * The names of its contributors can not be
#    used to endorse or promote products derived from this software without
#    specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

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

    label = [drawer.math_symbols('{^{b^c}p_{r^j}}(x)'),
             drawer.math_symbols('{^{b^c}p_{r^j}}(y)'),
             drawer.math_symbols('{^{b^c}p_{r^j}}(z)')]

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

    label = [drawer.math_symbols(r'{^{b^c}_{r^j}{q}(x)}'),
             drawer.math_symbols(r'{^{b^c}_{r^j}{q}(y)}'),
             drawer.math_symbols(r'{^{b^c}_{r^j}{q}(z)}'),
             drawer.math_symbols(r'{^{b^c}_{r^j}{q}(w)}')]

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
    ax.plot(np.array(data), c=figure_params_colors[0], label=drawer.math_symbols('{^{b^c}t_{r^j}}'),
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
    trans_dict = get_sensor_fields(['CalibParam', 'EXTRI', 'POS_RjInBc'])
    quat_dict = get_sensor_fields(['CalibParam', 'EXTRI', 'SO3_RjToBc'])
    time_dict = get_sensor_fields(['CalibParam', 'TEMPORAL', 'TIME_OFFSET_RjToBc'], True)

    num = len(trans_dict)

    for key in trans_dict:
        drawer.set_fig_size(16.0, 5.0)
        fig, axs = plt.subplots(3, 3, sharex=True)
        draw_trans([axs[0, 0], axs[1, 0], axs[2, 0]], trans_dict[key])
        draw_quat([axs[0, 1], axs[1, 1], axs[2, 1], axs[0, 2]], quat_dict[key])
        draw_time(axs[1, 2], time_dict[key])

        drawer.erase_ax(axs[2, 2])

        fig.suptitle('Topic: \'' + key + '\'')

        drawer.show_figure(figure_save_dir + '/radar' + key.replace("/", "_") + '.png')
