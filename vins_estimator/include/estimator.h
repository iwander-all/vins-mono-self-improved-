#ifndef _ESTIMATOR_H_
#define _ESTIMATOR_H_

//#include "parameters.h"
#include "feature_manager.h"
//#include "utility/utility.h"
//#include "utility/tic_toc.h"
//#include "initial/solve_5pts.h"
//#include "initial/initial_sfm.h"
//#include "initial/initial_alignment.h"
//#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include "backend.h"
#include "initial.h"

//#include <unordered_map>
//#include <queue>
//#include <opencv2/core/eigen.hpp>

class Backend;

class Estimator
{
  public:
    Estimator();
    /**
     * @brief set parameters from yaml
     */
    void setParameter();


    // ********************interface********************
    /**
     * @brief pre-integrate IMU
     * @param Input  double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity
     */
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

    /**
     * @brief main process of vio
     * @param Input  const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header
     */    
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);
   
    /**
     * @brief set relocolization frames
     * @param Input  double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r
     */    
    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);


    // ********************nornal functions********************
    /**
     * @brief inite all values for reboot
     */  
    void clearState();

    /**
     * @brief calibrate the rotation from cemera to IMU
     */  
    void calibrationExRotation();

    // ********************initial function groups********************
    /**
     * @brief initialize the system
     * @param Input  const std_msgs::Header &header
     */ 
    void initial(const std_msgs::Header &header);

  public:
    Backend backend;
    Initial intializer;

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

    // @param status flags
    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;

    // @param status flags
    bool first_imu;
    //bool is_valid, is_key;// @param seems useless, TODO can be deleted
    bool failure_occur;

    // @param at first, it is 0, after IMU-visual align, it is gc0, after intial, it is gw
    Vector3d g;

    // @param seems useless, TODO can be deleted
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    // @param status value: camera->IMU
    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];

    // @param status value: PVQ/bias in world
    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];
    double td;

    // @param temp buffer
    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;

    // @param status value: pre-integration bk->bk+1
    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    // @param buffer
    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    // @param frame count(max=10)
    int frame_count;

    // @param sum_of_outlier: no use, TODO can be deleted
    // @param ssum_of_back: how many second new frames are marged
    // @param sum_of_front: how many old frames are marged
    // @param sum_of_invalid: no use, TODO can be deleted
    //int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    // @param status value: all features
    FeatureManager f_manager;

    // @param function value: estimate motion
    MotionEstimator m_estimator;

    InitialEXRotation initial_ex_rotation;

    //vector<Vector3d> point_cloud;//no use, TODO can be deleted
    //vector<Vector3d> margin_cloud;//no use, TODO can be deleted
    vector<Vector3d> key_poses;
    double initial_timestamp;

    int loop_window_index;//TODO delete

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    // @param relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];

    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;
};

#endif