// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

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
