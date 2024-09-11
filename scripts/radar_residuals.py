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
radar_weight = 1.0
figure_save_dir = '/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/figures'

figure_color_alpha = 1.0
figure_marker_size = 3


def recovery_weight(data, weight):
    for i in range(len(data)):
        data[i][0] /= weight
    return data


def draw_pts_residuals(ax, data, title, set_left_ylable, set_right_ylabel):
    # hist
    ax.hist([e[0] for e in data], bins=30, density=False, facecolor="tab:blue", edgecolor="white",
            alpha=figure_color_alpha, zorder=1)
    # ax.ticklabel_format(style='sci', scilimits=(-1, 4), axis='y')
    drawer.set_sci_label(ax)
    ax.set_xlabel('PTS Distance $(m)$')
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
    drawer.set_yticks(ax_twinx, 0.0, 1.0, 5)
    drawer.add_grids(ax_twinx)
    drawer.set_label_decimal(ax_twinx, "%.3f", 'x')
    if set_right_ylabel:
        ax_twinx.set_ylabel("Probability")


if __name__ == '__main__':
    drawer.set_fig_size(15.0, 4.0)

    fig, axs = plt.subplots(1, len(residuals_files))
    for i in range(len(residuals_files)):
        fullname = params_file_dir + '/' + residuals_files[i]
        data = get_array_fields(fullname, ['residuals'])['RadarFactor']
        radar_data = []
        for elem in data:
            radar_data.append([elem['r0c0']])
        radar_data = recovery_weight(radar_data, radar_weight)
        draw_pts_residuals(
            axs[i], recovery_weight(radar_data, radar_weight),
            r"${r}_{ij}^{mn}(c)$ in BO " + str(i + 1), i == 0, i == len(residuals_files) - 1)

    drawer.show_figure(figure_save_dir)
