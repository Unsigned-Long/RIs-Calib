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

import json

from plt_utils import drawer
import matplotlib.pyplot as plt
from helper import get_array_fields
import numpy as np
from scipy.stats import norm

params_file_dir = '/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/lm_equ_graph'
residuals_files = ['residuals_0.json', 'residuals_1.json', 'residuals_2.json']
acce_weight = 1.0
gyro_weight = 1.0
figure_save_dir = '/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/figures'

figure_color_alpha = 1.0
figure_marker_size = 3


def recovery_weight(data, weight):
    for i in range(len(data)):
        data[i][0] /= weight
        data[i][1] /= weight
        data[i][2] /= weight

    return data


def draw_single_param(ax, data, title, set_left_ylable, set_right_ylabel):
    # hist
    ax.hist(data, bins=30, density=False, facecolor="tab:blue", edgecolor="white",
            alpha=figure_color_alpha, zorder=1)
    # ax.ticklabel_format(style='sci', scilimits=(-1, 4), axis='y')
    drawer.set_sci_label(ax)
    if set_left_ylable:
        ax.set_ylabel("Frequency")

    # norm distribution
    ax_twinx = ax.twinx()
    (loc, scale) = norm.fit(data)
    x = np.linspace(norm.ppf(1E-10, loc, scale), norm.ppf(1 - 1E-10, loc, scale), 300)
    ax_twinx.plot(
        x, norm.pdf(x, loc, scale) * scale, 'r-', lw=2, alpha=figure_color_alpha, zorder=2
    )
    ax.set_title(title)
    drawer.set_yticks(ax_twinx, 0.0, 1.0, 2)
    drawer.add_grids(ax_twinx)
    drawer.set_label_decimal(ax_twinx, "%.3f", 'x')
    if set_right_ylabel:
        ax_twinx.set_ylabel("Probability")


if __name__ == '__main__':
    drawer.set_fig_size(15.0, 12.0)

    fig, axs = plt.subplots(6, len(residuals_files))
    for i in range(len(residuals_files)):
        fullname = params_file_dir + '/' + residuals_files[i]
        acce_data_raw = get_array_fields(fullname, ['residuals'])['IMUAcceFactor']
        acce_data = []
        for elem in acce_data_raw:
            acce_data.append([elem['r0c0'], elem['r1c0'], elem['r2c0']])
        acce_data = recovery_weight(acce_data, acce_weight)

        gyro_data_raw = get_array_fields(fullname, ['residuals'])['IMUGyroFactor']
        gyro_data = []
        for elem in gyro_data_raw:
            gyro_data.append([elem['r0c0'], elem['r1c0'], elem['r2c0']])
        gyro_data = recovery_weight(gyro_data, gyro_weight)

        draw_single_param(
            axs[0, i], [e[0] for e in gyro_data],
            r"${r}^{i}(\omega_{x})$ in BO " + str(i + 1), i == 0, i == len(residuals_files) - 1)

        draw_single_param(
            axs[1, i], [e[1] for e in gyro_data],
            r"${r}^{i}(\omega_{y})$ in BO " + str(i + 1), i == 0, i == len(residuals_files) - 1)

        draw_single_param(
            axs[2, i], [e[2] for e in gyro_data],
            r"${r}^{i}(\omega_{z})$ in BO " + str(i + 1), i == 0, i == len(residuals_files) - 1)

        draw_single_param(
            axs[3, i], [e[0] for e in acce_data],
            r"${r}^{i}(a_{x})$ in BO " + str(i + 1), i == 0, i == len(residuals_files) - 1)

        draw_single_param(
            axs[4, i], [e[1] for e in acce_data],
            r"${r}^{i}(a_{y})$ in BO " + str(i + 1), i == 0, i == len(residuals_files) - 1)

        draw_single_param(
            axs[5, i], [e[2] for e in acce_data],
            r"${r}^{i}(a_{z})$ in BO " + str(i + 1), i == 0, i == len(residuals_files) - 1)
    drawer.show_figure(figure_save_dir + '/imu_residuals.png')
