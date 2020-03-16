#include "../include/initial.h"
#include "../include/estimator.h" 


bool Initial::initialStructure(Estimator *estimator)
{
    // 1.check imu observibility
    //TicToc t_sfm;
    //if (!checkIMUObservibility())
        //return false;

    // 2.build sfm features which are used in initialization
    vector<SFMFeature> sfm_f;
    buildSFMFeature(estimator, sfm_f);

    // 3.find frame i in sliding window,whose average_parallax is above threshold
    // and find the relative R & T(l<-11) between frame i and the newest frame(No.11)
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(estimator,relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }

    // 4.solve SfM problems to get the rotation/translation for all frames and landmarks
    // in frame l coordinate system
    GlobalSFM sfm;
    Quaterniond Q[estimator->frame_count + 1];
    Vector3d T[estimator->frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;

    if(!sfm.construct(estimator->frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        estimator->marginalization_flag = estimator->MARGIN_OLD;
        return false;
    }

    // 5.solve pnp for all frame
    if(!solvePnPForAllFrame(estimator,Q,T,sfm_tracked_points))
        return false;

    // 6. align visual and IMU for initialization
    if (visualInitialAlign(estimator))
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }
}

bool Initial::checkIMUObservibility(Estimator *estimator)
{
    map<double, ImageFrame>::iterator frame_it;
    Vector3d sum_g;
    for (frame_it = estimator->all_image_frame.begin(), frame_it++; frame_it != estimator->all_image_frame.end(); frame_it++)
    {
        double dt = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        sum_g += tmp_g;
    }
    Vector3d aver_g;
    aver_g = sum_g * 1.0 / ((int)estimator->all_image_frame.size() - 1);
    double var = 0;
    for (frame_it = estimator->all_image_frame.begin(), frame_it++; frame_it != estimator->all_image_frame.end(); frame_it++)
    {
        double dt = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        //cout << "frame g " << tmp_g.transpose() << endl;
    }
    var = sqrt(var / ((int)estimator->all_image_frame.size() - 1));
    //ROS_WARN("IMU variation %f!", var);
    if(var < 0.25)
    {
        ROS_INFO("IMU excitation not enouth!");
        return false;
    }
    return true;
}

void Initial::buildSFMFeature(Estimator *estimator, vector<SFMFeature> &sfm_f)
{
    for (auto &it_per_id : estimator->f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
}

bool Initial::relativePose(Estimator *estimator, Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = estimator->f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && estimator->m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

bool Initial::solvePnPForAllFrame(Estimator *estimator, Quaterniond Q[], Vector3d T[],map<int, Vector3d> &sfm_tracked_points)
{
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = estimator->all_image_frame.begin( );
    for (int i = 0; frame_it != estimator->all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == estimator->Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > estimator->Headers[i].stamp.toSec())
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
}

bool Initial::visualInitialAlign(Estimator *estimator)
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(estimator->all_image_frame, estimator->Bgs, estimator->g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    recoverStatusValuesFromInitial(estimator,x);

    ROS_DEBUG_STREAM("g0     " << estimator->g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(estimator->Rs[0]).transpose()); 

    return true;
}

void Initial::recoverStatusValuesFromInitial(Estimator *estimator, const VectorXd &x)
{
    for (int i = 0; i <= estimator->frame_count; i++)
    {
        Matrix3d Ri = estimator->all_image_frame[estimator->Headers[i].stamp.toSec()].R;
        Vector3d Pi = estimator->all_image_frame[estimator->Headers[i].stamp.toSec()].T;
        estimator->Ps[i] = Pi;
        estimator->Rs[i] = Ri;
        estimator->all_image_frame[estimator->Headers[i].stamp.toSec()].is_key_frame = true;
    }

    VectorXd dep = estimator->f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    estimator->f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    estimator->ric[0] = RIC[0];
    estimator->f_manager.setRic(estimator->ric);
    estimator->f_manager.triangulate(estimator->Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        estimator->pre_integrations[i]->repropagate(Vector3d::Zero(), estimator->Bgs[i]);
    }

    for (int i = estimator->frame_count; i >= 0; i--)
        estimator->Ps[i] = s * estimator->Ps[i] - estimator->Rs[i] * TIC[0] - (s * estimator->Ps[0] - estimator->Rs[0] * TIC[0]);

    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = estimator->all_image_frame.begin(); frame_i != estimator->all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            estimator->Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    for (auto &it_per_id : estimator->f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(estimator->g);
    double yaw = Utility::R2ypr(R0 * estimator->Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    estimator->g = R0 * estimator->g;

    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= estimator->frame_count; i++)
    {
        estimator->Ps[i] = rot_diff * estimator->Ps[i];
        estimator->Rs[i] = rot_diff * estimator->Rs[i];
        estimator->Vs[i] = rot_diff * estimator->Vs[i];
    }
}