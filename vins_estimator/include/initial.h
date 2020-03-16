#ifndef _INITIAL_H_
#define _INITIAL_H_

#include <vector>

#include "parameters.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"

using namespace std;

class Estimator;

class Initial
{
  public:
    // ********************initial function groups********************

    /**
     * @brief main pocess of initialization
     */ 
    bool initialStructure(Estimator *estimator);

    /**
     * @brief check IMU Observibility
     * TODO not necessary, can be deleted
     */    
    bool checkIMUObservibility(Estimator *estimator);

    /**
     * @brief build sfm_f for SfM
     * @param Input  vector<SFMFeature> &sfm_f
     * @param output vector<SFMFeature> &sfm_f
     */ 
    void buildSFMFeature(Estimator *estimator, vector<SFMFeature> &sfm_f);

    /**
     * @brief find base frame l in sliding windows and get relative rotation and translation 
     * between frame l and newest frame
     * @param Input  Matrix3d &relative_R, Vector3d &relative_T, int &l
     * @param output Matrix3d &relative_R, Vector3d &relative_T, int &l
     */ 
    bool relativePose(Estimator *estimator, Matrix3d &relative_R, Vector3d &relative_T, int &l);

    /**
     * @brief get the rotation and translation for all frames and 3D coordinates of all features 
     * in frame l without scaler
     * @param Input  Quaterniond Q[], Vector3d T[],map<int, Vector3d> &sfm_tracked_points
     */ 
    bool solvePnPForAllFrame(Estimator *estimator, Quaterniond Q[], Vector3d T[],map<int, Vector3d> &sfm_tracked_points);

    /**
     * @brief loosely coupled IMU-visual initialization
     */ 
    bool visualInitialAlign(Estimator *estimator);

    /**
     * @brief get the rotation and translation for all frames and 3D coordinates of all features 
     * in frame world with scaler
     * @param Input  const VectorXd &x
     */
    void recoverStatusValuesFromInitial(Estimator *estimator, const VectorXd &x);

  public:
    Estimator *estimator;

    // @param status flags
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