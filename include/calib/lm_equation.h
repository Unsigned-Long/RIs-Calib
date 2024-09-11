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

#ifndef RIS_CALIB_LM_EQUATION_H
#define RIS_CALIB_LM_EQUATION_H

#include "ceres/ceres.h"
#include "opencv2/imgproc.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "util/utils.hpp"

namespace ns_ris {
    struct CeresFactor {
    private:
        ceres::CostFunction *_costFunction;
        std::vector<double *> _paramBlocks;
        std::size_t _costFunctorHashCode;

        // param address, jacobian matrix
        std::map<const double *, Eigen::MatrixXd> jacobians;
        Eigen::VectorXd residuals;

    public:
        CeresFactor(ceres::CostFunction *costFunction, const std::vector<double *> &paramBlocks,
                    std::size_t costFunctorHashCode);

        void Evaluate(const std::map<const double *, std::string> &targetParams);

        [[nodiscard]] const std::map<const double *, Eigen::MatrixXd> &GetJacobians() const;

        [[nodiscard]] const Eigen::VectorXd &GetResiduals() const;

        [[nodiscard]] size_t GetCostFunctorHashCode() const;
    };

    struct LMEquation {
    private:
        Eigen::MatrixXd _hMat;
        Eigen::VectorXd _bVec;

        // param name, param, dime
        std::vector<std::pair<std::string, std::size_t>> _paramDesc;

        std::map<std::size_t, Eigen::aligned_vector<Eigen::VectorXd>> _residualsMap;

    public:
        LMEquation(Eigen::MatrixXd hMat, Eigen::VectorXd bVec,
                   const std::vector<std::pair<std::string, std::size_t>> &paramDesc,
                   const std::map<std::size_t, Eigen::aligned_vector<Eigen::VectorXd>> &residualsMap);

        [[nodiscard]] const LMEquation &
        SaveParamBlockOrderListToDisk(const std::string &filepath, CerealArchiveType::Enum archiveType) const;

        [[nodiscard]] cv::Mat EquationGraph() const;

        [[nodiscard]] const std::map<std::size_t, Eigen::aligned_vector<Eigen::VectorXd>> &GetResidualsMap() const;

    protected:
        [[nodiscard]] const LMEquation &
        SaveParamBlockOrderListToDisk(const std::string &filepath, const Eigen::MatrixXd &hMat,
                                      const Eigen::VectorXd &bVec, CerealArchiveType::Enum archiveType) const;

        [[nodiscard]] cv::Mat EquationGraph(const Eigen::MatrixXd &hMat, const Eigen::VectorXd &bVec) const;
    };
}


#endif //RIS_CALIB_LM_EQUATION_H
