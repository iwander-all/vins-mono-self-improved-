#ifndef _FEATURE_TRACKER_H_
#define _FEATURE_TRACKER_H_

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;


/**
 * @brief check whether the pixel coordinates of tracking features in border or not
 * @param Input cv::Point2f &pt
 */
bool inBorder(const cv::Point2f &pt);

/**
 * @brief delete items in output value according to the input value
 * @param Input  vector<uchar> status
 * @param Output vector<cv::Point2f> &v
*/
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);

/**
 * @brief delete items in output value according to the input value
 * @param Input  vector<uchar> status
 * @param Output vector<int> &v
 */
void reduceVector(vector<int> &v, vector<uchar> status);


/**
 * @brief Class of FeatureTracker
 */
class FeatureTracker
{
  public:
    FeatureTracker();

    /**
     * @brief eualize the image 
     * @param Input  const cv::Mat &_img
     * @param Output cv::Mat &img
     */
    void equalize(const cv::Mat &_img,cv::Mat &img);

    /**
     * @brief track existing features and delete failures 
     */
    void flowTrack();

    /**
     * @brief track new features in forward image
     */
    void trackNew();

    /**
     * @brief main process of feature tracker
     * @param Input  const cv::Mat &_img
     * @param Input  double _cur_time
     */
    void readImage(const cv::Mat &_img,double _cur_time);

    /**
     * @brief rank features according to their existing times and setMask for high rank features
     * to avoid crowded layout of features
     */
    void setMask();

    /**
     * @brief add new tracked features into buffer
     */
    void addPoints();

    /**
     * @brief update ID for features
     */
    bool updateID(unsigned int i);

    /**
     * @brief read cameras' parameters
     * @param Input  const string &calib_file
     */
    void readIntrinsicParameter(const string &calib_file);

    /**
     * @brief show undistortion
     * TODO not important ,can be deleted
     * @param Input  const string &name
     */
    void showUndistortion(const string &name);

    /**
     * @brief use Fundamental matrix to delete outliers
     */
    void rejectWithF();

    /**
     * @brief get undistorted normalized coordinates of features
     * @param Input  const string &name
     */
    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;

    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;

    camodocal::CameraPtr m_camera;
    
    double cur_time;
    double prev_time;

    static int n_id;
};

#endif