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
import matplotlib.pyplot as plt
import numpy as np

params_file = '/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/params_iter/iter_info.csv'
figure_save_dir = '/home/csl/ros_ws/RIs-Calib/src/ris_calib/output/simu3/figures'

figure_params_colors = ['r', 'g', 'b']
figure_color_alpha = 1.0
figure_marker_size = 3

figure_colors = [
    ['#FF3976', '#FA5D5D', '#FF7C61', '#FE9B66', '#FFC476'],
    ['#0320FF', '#0259E6', '#0AA8FC', '#02D3E6', '#03FFD1'],
    ['#3F4036', '#58594F', '#8B8C81', '#BFBFBA', '#D9D9D2'],
    ['#003840', '#005A5B', '#007369', '#008C72', '#02A676']
]


def read_iter_info(filename):
    file = open(filename, "r")
    lines = file.readlines()

    cost = []
    gradient = []
    tr_radius = []

    for line in lines[1:]:
        elems = line.split(',')
        cost.append(float(elems[1]))
        gradient.append(float(elems[2]))
        tr_radius.append(float(elems[3]))

    return [cost, gradient, tr_radius]


def draw_iter_info(axs, cost, gradient, tr_radius, batch_optimization, titles, ylabels, figure_colors, desc):
    for i in range(len(batch_optimization) - 1):
        axs[0].plot(
            np.arange(batch_optimization[i], batch_optimization[i + 1]),
            cost[batch_optimization[i]:batch_optimization[i + 1]], marker='o', ms=figure_marker_size,
            c=figure_colors[i], label=desc + ' BO ' + str(i + 1), alpha=figure_color_alpha,
        )
        axs[1].plot(
            np.arange(batch_optimization[i], batch_optimization[i + 1]),
            gradient[batch_optimization[i]:batch_optimization[i + 1]], marker='o', ms=figure_marker_size,
            c=figure_colors[i], label=desc + ' BO ' + str(i + 1), alpha=figure_color_alpha,
        )
        axs[2].plot(
            np.arange(batch_optimization[i], batch_optimization[i + 1]),
            tr_radius[batch_optimization[i]:batch_optimization[i + 1]], marker='o', ms=figure_marker_size,
            c=figure_colors[i], label=desc + ' BO ' + str(i + 1), alpha=figure_color_alpha,
        )

    # for i in range(3):
    #     # log mapping
    #     if i != 2:
    #         axs[i].set_yscale("log", basey=10)
    #
    #     axs[i].set_title(titles[i])
    #     axs[i].set_xlabel('Iterations')
    #     axs[i].set_ylabel(ylabels[i])
    #     axs[i].legend()
    #     axs[i].grid(ls='-.', alpha=0.5)


def draw_cost(ax, data, splitor):
    for i in range(len(splitor) - 1):
        ax.plot(np.arange(splitor[i], splitor[i + 1]),
                data[splitor[i]:splitor[i + 1]], marker='o', ms=figure_marker_size,
                label='BO ' + str(i + 1), alpha=figure_color_alpha)
    ax.set_yscale("log", base=10)
    ax.legend()
    ax.set_title('Objective Function Variation')
    drawer.add_grids(ax)
    ax.set_xlabel('Iterations')
    ax.set_ylabel(r'Log Objective Function $(\log_{10}r)$')


def draw_gradient(ax, data, splitor):
    for i in range(len(splitor) - 1):
        ax.plot(np.arange(splitor[i], splitor[i + 1]),
                data[splitor[i]:splitor[i + 1]], marker='o', ms=figure_marker_size,
                label='BO ' + str(i + 1), alpha=figure_color_alpha)
    ax.set_yscale("log", base=10)
    ax.legend()
    ax.set_title('Gradient Norm Variation')
    drawer.add_grids(ax)
    ax.set_xlabel('Iterations')
    ax.set_ylabel(r'Log Gradient Norm $(\log_{10}|g|)$')


def draw_tr_radius(ax, data, splitor):
    for i in range(len(splitor) - 1):
        ax.plot(np.arange(splitor[i], splitor[i + 1]),
                data[splitor[i]:splitor[i + 1]], marker='o', ms=figure_marker_size,
                label='BO ' + str(i + 1), alpha=figure_color_alpha)
    ax.legend()
    ax.set_title('Trust Region Radius Variation')
    drawer.add_grids(ax)
    ax.set_xlabel('Iterations')
    ax.set_ylabel(r'Trust Region Radius $(\log_{10}1/\lambda)$')


if __name__ == '__main__':
    cost, gradient, tr_radius = read_iter_info(params_file)

    splitor = []
    for i in range(len(tr_radius)):
        if tr_radius[i] == 10000.0:
            splitor.append(i)
    splitor.append(len(tr_radius))

    drawer.set_fig_size(15.0, 5.0)
    fig, axs = plt.subplots(1, 3, sharex=True)

    draw_cost(axs[0], cost, splitor)
    draw_gradient(axs[1], gradient, splitor)
    draw_tr_radius(axs[2], tr_radius, splitor)

    drawer.show_figure(figure_save_dir + '/iter_info.png')
