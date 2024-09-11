// RIs-Calib: An Open-Source Spatiotemporal Calibrator for Multiple 3D Radars and IMUs Based on Continuous-Time Estimation
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/RIs-Calib.git
//
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
//
// Purpose: See .h/.hpp file.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RIS_CALIB_EQUATION_VERIFY_HPP
#define RIS_CALIB_EQUATION_VERIFY_HPP

#include "ctraj/core/simu_trajectory.h"
#include "radar_model.hpp"

namespace ns_ris {

    struct EquationVerify {
    public:
        static Sophus::SE3d GeneratePoseBias() {
            constexpr double DEG_TO_RAD = M_PI / 180.0;
            static std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> aRand(-180, 180), pRand(-0.5, 0.5);
            auto a1 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 0, 1));
            auto a2 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(0, 1, 0));
            auto a3 = Eigen::AngleAxisd(aRand(engine) * DEG_TO_RAD, Eigen::Vector3d(1, 0, 0));
            return {(a1 * a2 * a3).toRotationMatrix(), Eigen::Vector3d(pRand(engine), pRand(engine), pRand(engine))};
        }

        static void Equation1() {
            auto trajBtoB0 = ns_ctraj::SimuWaveMotion2<4>(2.0, 0.5, 0.0, 2 * M_PI, 1000.0);
            auto BtoB0 = *trajBtoB0.GetTrajectory();
            auto SE3_RtoB = GeneratePoseBias();
            auto RtoB0 = BtoB0 * SE3_RtoB;
            Eigen::Vector3d TinB0(1, 3, 5);
            auto TinR = SE3_RtoB.inverse() * (!BtoB0) * Sophus::SE3d(Sophus::SO3d(), TinB0);
            for (const auto &item: BtoB0.Sampling(0.1)) {
                double t = item.timeStamp;
                {
                    // right
                    Eigen::Vector3d v1 =
                            -Sophus::SO3d::hat(BtoB0.Pose(t).so3() * SE3_RtoB.translation()) *
                            BtoB0.AngularVeloInRef(t);
                    Eigen::Vector3d v2 = BtoB0.LinearVeloInRef(t);
                    Sophus::SO3d SO3_B0ToR = SE3_RtoB.so3().inverse() * BtoB0.Pose(t).so3().inverse();
                    Eigen::Vector3d v3 = SO3_B0ToR * (v1 + v2);

                    // left
                    Eigen::Vector3d v4 = SO3_B0ToR * RtoB0.LinearVeloInRef(t);

                    std::cout << "v4: " << v4.transpose() << std::endl;
                    std::cout << "v3: " << v3.transpose() << std::endl << std::endl;
                }

                // left
                {
                    Eigen::Vector3d v1 = TinR.LinearVeloInRef(t);

                    // right
                    // Eigen::Vector3d v2 = Sophus::SO3d::hat(TinR.Pose(t).translation()) *
                    //                      (SE3_RtoB.so3().inverse() * BtoB0.Pose(t).so3().inverse() *
                    //                       BtoB0.AngularVeloInRef(t));
                    Eigen::Vector3d v3 = SE3_RtoB.so3().inverse() *
                                         (Sophus::SO3d::hat(SE3_RtoB.translation()) *
                                          (BtoB0.Pose(t).so3().inverse() * BtoB0.AngularVeloInRef(t)));

                    Eigen::Vector3d v4 =
                            SE3_RtoB.so3().inverse() * (BtoB0.Pose(t).so3().inverse() * BtoB0.LinearVeloInRef(t));

                    Eigen::Vector3d v5 = v3 - v4;
                    std::cout << "v1: " << v1.transpose() << std::endl;
                    std::cout << "v5: " << v5.transpose() << std::endl;
                    std::cout << std::endl << std::endl << std::endl;
                }
            }
        }

        static Eigen::aligned_vector<std::pair<double, Eigen::aligned_vector<RadarTarget::Ptr>>>
        SimuRadarData(ns_ctraj::Trajectory<4> &traj, const Sophus::SE3d &bias) {
            std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
            std::uniform_real_distribution<double> u(-3, 3);
            Eigen::aligned_vector<std::pair<double, Eigen::aligned_vector<RadarTarget::Ptr>>> data;
            std::vector<Eigen::Vector3d> targets;
            for (int i = 0; i < 100; ++i) {
                Eigen::Vector3d tar(u(engine), u(engine), u(engine));
                targets.push_back(tar);
            }
            for (const auto &item: traj.Sampling(0.1)) {
                double t = item.timeStamp;
                std::pair<double, Eigen::aligned_vector<RadarTarget::Ptr>> obv;
                obv.first = t;
                for (const auto &tar: targets) {
                    obv.second.push_back(RadarTarget::Create(t, traj.RadarStaticMeasurement(t, tar, bias)));
                }
                data.push_back(obv);
            }
            return data;
        }

        static void Equation2() {
            auto trajBtoB0 = ns_ctraj::SimuEightShapeMotion<4>(5.0, 4.0, 0.5);
            auto BtoB0 = *trajBtoB0.GetTrajectory();
            auto SE3_RtoB = GeneratePoseBias();
            auto RtoB0 = BtoB0 * SE3_RtoB;

            auto radarMes = SimuRadarData(BtoB0, SE3_RtoB);

            for (const auto &item: radarMes) {
                double t = item.first;
                std::cout << "t: " << t << std::endl;

                // pred
                Sophus::SO3d SO3_B0ToR = SE3_RtoB.so3().inverse() * BtoB0.Pose(t).so3().inverse();
                Eigen::Vector3d v1 = SO3_B0ToR * RtoB0.LinearVeloInRef(t);


                // estimation
                const auto &obv = item.second;
                Eigen::VectorXd lVec(obv.size());
                Eigen::MatrixXd BMat(obv.size(), 3);
                for (int i = 0; i < static_cast<int>(obv.size()); ++i) {
                    const auto &tar = obv.at(i);
                    lVec(i) = tar->GetRadialVelocity();
                    BMat.block<1, 3>(i, 0) = tar->GetTargetXYZ().normalized().transpose();
                }
                Eigen::Vector3d v2 = (BMat.transpose() * BMat).inverse() * BMat.transpose() * lVec;

                std::cout << "v1: " << v1.transpose() << std::endl;
                std::cout << "v2: " << v2.transpose() << std::endl << std::endl;
            }
        }
    };
}

#endif //RIS_CALIB_EQUATION_VERIFY_HPP
