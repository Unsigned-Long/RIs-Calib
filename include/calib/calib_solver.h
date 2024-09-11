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

#ifndef RIS_CALIB_CALIB_SOLVER_H
#define RIS_CALIB_CALIB_SOLVER_H

#include <utility>

#include "calib/calib_param_manager.h"
#include "calib/calib_data_manager.h"
#include "ctraj/core/trajectory.h"
#include "ceres/ceres.h"
#include "calib/spline_estimator.h"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "factor/imu_acce_factor.hpp"
#include "factor/imu_gyro_factor.hpp"
#include "factor/radar_factor.hpp"
#include "tiny-viewer/core/multi_viewer.h"

namespace ns_ris {

    class CalibSolver {
    public:
        using Ptr = std::shared_ptr<CalibSolver>;
        using Trajectory = ns_ctraj::Trajectory<Configor::Prior::SplineOrder>;

        const static std::string VIEW_SENSORS, VIEW_SPLINE;

    private:
        CalibDataManager::Ptr _dataManager;

        CalibParamManager::Ptr _paramManager;

        Trajectory::Ptr _trajectory;

        ceres::Solver::Options _ceresOption;

        ns_viewer::MultiViewer::Ptr _viewer;

        bool _solveFinished;

    public:
        explicit CalibSolver(CalibDataManager::Ptr calibDataManager, CalibParamManager::Ptr calibParamManager);

        static CalibSolver::Ptr
        Create(const CalibDataManager::Ptr &calibDataManager, const CalibParamManager::Ptr &calibParamManager);

        void Process();

        void VisualizationSensorSuite() const;

        void VisualizationBSplines() const;

        void Visualization() const;

        void SaveBSplines(int hz = 100) const;

        virtual ~CalibSolver();

    protected:
        void Initialization();

        /**
         * the function to perform batch optimization
         *
         * @param radarOptOption
         * options: 'OPT_SO3', 'OPT_VEL',
         *          'OPT_SO3_RtoB', 'OPT_POS_RinB', 'OPT_TIME_OFFSET_RtoB'
         *
         * @param acceOptOption
         * options: 'OPT_SO3', 'OPT_VEL',
         *          'OPT_ACCE_BIAS', 'OPT_ACCE_MAP_COEFF', 'OPT_GRAVITY'
         *
         * @param gyroOptOption
         * options: 'OPT_SO3', 'OPT_VEL',
         *          'OPT_GYRO_BIAS', 'OPT_GYRO_MAP_COEFF', 'OPT_SO3_AtoG'
         */
        void BatchOptimization(int radarOptOption, int acceOptOption, int gyroOptOption);

        void SaveEquationGraph(const SplineEstimator::Ptr &estimator);

        static std::map<const double *, std::string> GetCTrajSO3KnotAddressWithDesc(const Trajectory::Ptr &traj);

        static std::map<const double *, std::string> GetCTrajVelKnotAddressWithDesc(const Trajectory::Ptr &traj);
    };


    struct RIsCeresDebugCallBack : public ceres::IterationCallback {
        CalibParamManager::Ptr _calibParamManager;

        explicit RIsCeresDebugCallBack(CalibParamManager::Ptr calibParamManager);

        static auto Create(const CalibParamManager::Ptr &calibParamManager);

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override;
    };

    struct RIsCeresViewerCallBack : public ceres::IterationCallback {
        using Trajectory = ns_ctraj::Trajectory<Configor::Prior::SplineOrder>;

        CalibParamManager::Ptr _parMagr;
        ns_viewer::MultiViewer::Ptr _viewer;
        RIsCeresViewerCallBack::Trajectory::Ptr _traj;
        std::vector<std::size_t> _idVec;

        explicit RIsCeresViewerCallBack(CalibParamManager::Ptr calibParamManager,
                                        ns_viewer::MultiViewer::Ptr viewer,
                                        RIsCeresViewerCallBack::Trajectory::Ptr traj);

        static auto Create(const CalibParamManager::Ptr &calibParamManager,
                           const ns_viewer::MultiViewer::Ptr &viewer,
                           const RIsCeresViewerCallBack::Trajectory::Ptr &traj);

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override;

        std::vector<std::size_t> AddEntitiesToSensorViewer();

        std::vector<std::size_t> AddEntitiesToSplineViewer();
    };

}

#endif //RIS_CALIB_CALIB_SOLVER_H
