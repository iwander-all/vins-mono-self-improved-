#ifndef _BACKEND_H_
#define _BACKEND_H_

#include <ceres/ceres.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <vector>

#include "parameters.h"
#include "utility/tic_toc.h"
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"
#include "factor/integration_base.h"

using namespace std;
//#include "estimator.h"

class Estimator;

class Backend
{
  public:
    Backend();

    void clearState();

    /**
     * @brief backend
     */ 
    void backend(Estimator *estimator);

    /**
     * @brief main pocess of backend
     */ 
    void solveOdometry(Estimator *estimator);

    /**
     * @brief main pocess of solveOdometry
     */ 
    void optimization(Estimator *estimator);

    /**
     * @brief nonLinear Optimization
     * @param Input  ceres::Problem &problem,ceres::LossFunction *loss_function
     */ 
    void nonLinearOptimization(Estimator *estimator,ceres::Problem &problem,ceres::LossFunction *loss_function);

    /**
     * @brief change status values from vector to double in favor of ceres
     */ 
    void vector2double(Estimator *estimator);

    /**
     * @brief recover status values from double to vector
     */ 
    void double2vector(Estimator *estimator);

    /**
     * @brief detect failure
     */ 
    bool failureDetection(Estimator *estimator);

    /**
     * @brief marginizate old frames for big Hessian matrix
     */ 
    void margOld(Estimator *estimator,ceres::LossFunction *loss_function);

    /**
     * @brief marginizate new frames for big Hessian matrix
     */ 
    void margNew(Estimator *estimator);


    /**
     * @brief main process of sliding window for status valuee
     */ 
    void slideWindow(Estimator *estimator);

    /**
     * @brief marginizate new frames
     */ 
    void slideWindowNew(Estimator *estimator);

    /**
     * @brief marginizate old frames
     */ 
    void slideWindowOld(Estimator *estimator);

  public:
    Estimator *estimator;
    // @param temp values for ceres optimization
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1]; //TODO delete

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    // @param status flags
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
};

#endif