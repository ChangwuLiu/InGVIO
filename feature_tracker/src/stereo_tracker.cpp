#include "stereo_tracker.h"

#include "Color.h"

namespace feature_tracker
{
    unsigned long long int StereoTracker::n_id = 0;
    
    void StereoTracker::init()
    {
        param.readParameters(nh);
        
        readIntrinsicParameters();
        
        cam_left_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, param.image_left_topic, 50));
        cam_right_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, param.image_right_topic, 50));
        stereo_sub_.reset(new message_filters::Synchronizer<stereoSyncPolicy>(stereoSyncPolicy(50), *cam_left_sub_, *cam_right_sub_));
        stereo_sub_->registerCallback(boost::bind(&StereoTracker::stereo_callback, this, _1, _2));
        
        pub_stereo_feature = nh.advertise<feature_tracker::StereoFrame>("stereo_feature", 50);
        pub_match = nh.advertise<sensor_msgs::Image>("stereo_track", 50);
        
        first_image_flag = true;
        first_image_time = 0.0;
        last_image_time = 0.0;
        
        pub_this_frame = false;
        pub_count = 1;
    }
    
    void StereoTracker::readIntrinsicParameters()
    {
        ROS_INFO("[Stereo Tracker]: Reading parameter of left camera from: %s", param.cam_left_path.c_str());
        
        m_camera_left = camera_model::CameraFactory::instance()->generateCameraFromYamlFile(param.cam_left_path);
        
        ROS_INFO("[Stereo Tracker]: Reading parameter of right camera from: %s", param.cam_right_path.c_str());
        
        m_camera_right = camera_model::CameraFactory::instance()->generateCameraFromYamlFile(param.cam_right_path);        
    }
    
    void StereoTracker::stereo_callback(const sensor_msgs::Image::ConstPtr& cam_left_msg, const sensor_msgs::Image::ConstPtr& cam_right_msg)
    {
        TicToc stereo_timer;
        
        if (first_image_flag)
        {
            first_image_flag = false;
            first_image_time = cam_left_msg->header.stamp.toSec();
            last_image_time = cam_left_msg->header.stamp.toSec();
            return;
        }
        
        if (cam_left_msg->header.stamp.toSec() - last_image_time > 1.0 || cam_left_msg->header.stamp.toSec() < last_image_time)
        {
            ROS_WARN("[Stereo Tracker]: Image discontinue! Stereo tracker reset!");
            first_image_flag = true;
            first_image_time = 0.0;
            last_image_time = 0.0;
            pub_count = 1;
            pub_this_frame = false;
            return;
        }
        
        last_image_time = cam_left_msg->header.stamp.toSec();
        
        if (std::round(1.0*pub_count/(last_image_time-first_image_time)) <= param.freq)
        {
            pub_this_frame = true;
            
            if (std::fabs(1.0*pub_count/(last_image_time-first_image_time)-param.freq) < 0.01*param.freq)
            {
                first_image_time = last_image_time;
                pub_count = 0;
            }
        }
        else
        {
            pub_this_frame = false;
        }        
        
        cv_bridge::CvImageConstPtr left_ptr, right_ptr;
        if (cam_left_msg->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = cam_left_msg->header;
            img.height = cam_left_msg->height;
            img.width = cam_left_msg->width;
            img.is_bigendian = cam_left_msg->is_bigendian;
            img.step = cam_left_msg->step;
            img.data = cam_left_msg->data;
            img.encoding = "mono8";
            left_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            left_ptr = cv_bridge::toCvCopy(cam_left_msg, sensor_msgs::image_encodings::MONO8);
        if (cam_right_msg->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = cam_right_msg->header;
            img.height = cam_right_msg->height;
            img.width = cam_right_msg->width;
            img.is_bigendian = cam_right_msg->is_bigendian;
            img.step = cam_right_msg->step;
            img.data = cam_right_msg->data;
            img.encoding = "mono8";
            right_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            right_ptr = cv_bridge::toCvCopy(cam_right_msg, sensor_msgs::image_encodings::MONO8);
        
        processStereoImage(left_ptr->image, right_ptr->image, cam_left_msg->header.stamp.toSec());
        
        if (pub_this_frame)
        {
            ++pub_count;
            
            feature_tracker::StereoFrame::Ptr stereo_frame_ptr(new feature_tracker::StereoFrame);
            stereo_frame_ptr->header = cam_left_msg->header;
            
            for (unsigned int i = 0; i < ids.size(); ++i)
            {
                if (track_cnt[i] > 1)
                {
                    feature_tracker::StereoMeas::Ptr stereo_feature_ptr(new feature_tracker::StereoMeas);
                    
                    stereo_feature_ptr->id = ids[i];
                    stereo_feature_ptr->u0 = cur_left_un_pts[i].x;
                    stereo_feature_ptr->v0 = cur_left_un_pts[i].y;
                    stereo_feature_ptr->u1 = cur_right_un_pts[i].x;
                    stereo_feature_ptr->v1 = cur_right_un_pts[i].y;
                    
                    stereo_frame_ptr->stereo_features.push_back(*stereo_feature_ptr);
                }
            }
            
            if (stereo_frame_ptr->stereo_features.size() > 0)
                pub_stereo_feature.publish(stereo_frame_ptr);
            
            if (param.show_track)
            {
                cv::Mat track_img(param.row, 2*param.col, CV_8UC3);
                cv::cvtColor(left_ptr->image, track_img.colRange(0, param.col), CV_GRAY2BGR);
                cv::cvtColor(right_ptr->image, track_img.colRange(param.col, 2*param.col), CV_GRAY2BGR);
                
                for (unsigned int j = 0; j < cur_left_pts.size(); ++j)
                {
                    double len = std::min(1.0, 1.0*track_cnt[j]/param.window_size);
                    
                    cv::circle(track_img, cur_left_pts[j], 3, cv::Scalar(255*(1-len), 0, 255*len), 2);
                    cv::circle(track_img, cv::Point2f(cur_right_pts[j].x + param.col, cur_right_pts[j].y), 3, cv::Scalar(255*(1-len), 0, 255*len), 2);
                    
                    cv::line(track_img, cur_left_pts[j], cv::Point2f(cur_right_pts[j].x + param.col, cur_right_pts[j].y), cv::Scalar(255*(1-len), 0, 255*len), 2, 4);
                }
                
                cv_bridge::CvImage track_img_ros(cam_left_msg->header, "bgr8", track_img);
                
                pub_match.publish(track_img_ros.toImageMsg());
            }
        }
        
        if (param.show_timer)
        {
            double stereo_time = stereo_timer.toc();
            
            if (stereo_time <= param.timer_warning_thres)
                std::cout << color::setBlue << "[StereoTracker]: Stereo tracking processing time = " << stereo_time << " (ms) " << color::resetColor << std::endl;
            else
                std::cout << color::setRed << "[StereoTracker]: Stereo tracking processing time = " << stereo_time << " (ms) " << color::resetColor << std::endl;
        }
    }
    
    void StereoTracker::processStereoImage(const cv::Mat& _left_img, const cv::Mat& _right_img, double _cur_time)
    {
        cv::Mat left_img, right_img;
        cur_time = _cur_time;
        
        if (param.equalize)
        {
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            clahe->apply(_left_img, left_img);
            clahe->apply(_right_img, right_img);
        }
        else
        {
            left_img = _left_img;
            right_img = _right_img;
        }
        
        if (forw_left_img.empty())
        {
            prev_left_img = cur_left_img = forw_left_img = left_img;
            prev_right_img = cur_right_img = forw_right_img = right_img;
        }
        else
        {
            forw_left_img = left_img;
            forw_right_img = right_img;
        }
        
        forw_left_pts.clear();
        forw_right_pts.clear();
        
        if (cur_left_pts.size() > 0)
        {
            std::vector<uchar> status;
            std::vector<float> err;
            
            cv::calcOpticalFlowPyrLK(cur_left_img, forw_left_img, cur_left_pts, forw_left_pts, status, err, cv::Size(21, 21), 3);
            
            for (unsigned int i = 0; i < forw_left_pts.size(); ++i)
                if (status[i] && !inBorder(forw_left_pts[i]))
                    status[i] = 0;
                
            reduceTwinVector<cv::Point2f>(prev_left_pts, prev_right_pts, status);
            reduceTwinVector<cv::Point2f>(cur_left_pts, cur_right_pts, status);
            reduceVector<cv::Point2f>(forw_left_pts, status);
            reduceVector<unsigned long long int>(ids, status);
            // reduceTwinVector<cv::Point2f>(prev_left_un_pts, prev_right_un_pts, status);
            reduceTwinVector<cv::Point2f>(cur_left_un_pts, cur_right_un_pts, status);
            reduceVector<unsigned int>(track_cnt, status);
            
            if (forw_left_pts.size() > 0)
            {
                status.clear();
                err.clear();
                
                predictHomography(forw_left_pts, forw_right_pts);
            
                cv::calcOpticalFlowPyrLK(forw_left_img, forw_right_img, forw_left_pts, forw_right_pts, status, err, cv::Size(15, 15), 3, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
                for (unsigned int i = 0; i < forw_right_pts.size(); ++i)
                    if (status[i] && !inBorder(forw_right_pts[i]))
                        status[i] = 0;
                
                markStereoMatchOutlier(forw_left_pts, forw_right_pts, status);
            
                reduceTwinVector<cv::Point2f>(prev_left_pts, prev_right_pts, status);
                reduceTwinVector<cv::Point2f>(cur_left_pts, cur_right_pts, status);
                reduceTwinVector<cv::Point2f>(forw_left_pts, forw_right_pts, status);
                reduceVector<unsigned long long int>(ids, status);
                // reduceTwinVector<cv::Point2f>(prev_left_un_pts, prev_right_un_pts, status);
                reduceTwinVector<cv::Point2f>(cur_left_un_pts, cur_right_un_pts, status);
                reduceVector<unsigned int>(track_cnt, status);
            }    
        }
        
        for (auto& n: track_cnt)
            ++n;
        
        if (pub_this_frame)
        {
            rejectWithTwinRANSAC();
            
            setMask();
            
            int n_max_cnt = param.max_cnt - static_cast<int>(forw_left_pts.size());
            
            if (n_max_cnt > 0)
            {
                if (mask.empty())
                    ROS_WARN("[Stereo Tracker]: Mask is empty!");
                if (mask.type() != CV_8UC1)
                    ROS_WARN("[Stereo Tracker]: Mask type wrong!");
                if (mask.size() != forw_left_img.size())
                    ROS_WARN("[Stereo Tracker]: Mask size wrong!");

                n_left_pts.clear();
                n_right_pts.clear();
                
                cv::goodFeaturesToTrack(forw_left_img, n_left_pts, n_max_cnt, 0.01, param.min_dist, mask);
                
                
                if (n_left_pts.size() > 0)
                {
                    std::vector<uchar> status;
                    std::vector<float> err;
                    
                    predictHomography(n_left_pts, n_right_pts);
                    
                    cv::calcOpticalFlowPyrLK(forw_left_img, forw_right_img, n_left_pts, n_right_pts, status, err, cv::Size(15, 15), 3, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                    

                    for (unsigned int i = 0; i < n_right_pts.size(); ++i)
                        if (status[i] && !inBorder(n_right_pts[i]))
                            status[i] = 0;
                    
                    markStereoMatchOutlier(n_left_pts, n_right_pts, status);
                
                    reduceTwinVector<cv::Point2f>(n_left_pts, n_right_pts, status);
                }
                
            }
            else
            {
                n_left_pts.clear();
                n_right_pts.clear();
            }
            
            addPoints();
        }
        
        prev_left_img = cur_left_img;
        prev_right_img = cur_right_img;
        
        prev_left_pts = cur_left_pts;
        prev_right_pts = cur_right_pts;
        
        prev_left_un_pts = cur_left_un_pts;
        prev_right_un_pts = cur_right_un_pts;
        
        cur_left_img = forw_left_img;
        cur_right_img = forw_right_img;
        
        cur_left_pts = forw_left_pts;
        cur_right_pts = forw_right_pts;
        
        undistortedPoints();
        
        prev_time = cur_time;
        
        updateID();
        
        if (param.print_track_info)
            ROS_INFO("[Stereo Tracker]: Current feature pairs: %i, including newly detected pairs: %i", int(cur_left_pts.size()), int(n_left_pts.size()));
    }
    
    void StereoTracker::predictHomography(const std::vector<cv::Point2f>& left_pts, std::vector<cv::Point2f>& right_pts)
    {
        right_pts.clear();
        const Eigen::Matrix3d C_cl2cr = param.T_cr2i.linear().transpose()*param.T_cl2i.linear();
        
        for (unsigned int i = 0; i < left_pts.size(); ++i)
        {
            Eigen::Vector3d tmp_lpt;
            m_camera_left->liftProjective(Eigen::Vector2d(left_pts[i].x, left_pts[i].y), tmp_lpt);
            
            Eigen::Vector3d tmp_rpt = C_cl2cr*tmp_lpt;
            Eigen::Vector2d rpt;
            m_camera_right->spaceToPlane(tmp_rpt, rpt);
            right_pts.push_back(cv::Point2f(rpt.x(), rpt.y()));
        }    
    }
    
    void StereoTracker::publishStereoImg(const cv::Mat& left_img, const cv::Mat& right_img, const std::vector<cv::Point2f>& left_pts, const std::vector<cv::Point2f>& right_pts, const int row, const int col, bool pair_line)
    {
        cv::Mat track_img(row, 2*col, CV_8UC3);
        cv::cvtColor(left_img, track_img.colRange(0, col), CV_GRAY2BGR);
        cv::cvtColor(right_img, track_img.colRange(col, 2*col), CV_GRAY2BGR);
                
        for (unsigned int j = 0; j < left_pts.size(); ++j)
            cv::circle(track_img, left_pts[j], 3, cv::Scalar(255, 0, 0), 2);
        
        for (unsigned int j = 0; j < right_pts.size(); ++j)
            cv::circle(track_img, cv::Point2f(right_pts[j].x + col, right_pts[j].y), 3, cv::Scalar(255, 0, 0), 2);
        
        if (pair_line && left_pts.size() == right_pts.size())
            for (unsigned int j = 0; j < left_pts.size(); ++j)
                cv::line(track_img, left_pts[j], cv::Point2f(right_pts[j].x + col, right_pts[j].y), cv::Scalar(255, 0, 0), 2, 4); 
            
        cv::imshow("stereo_match", track_img);
        cv::waitKey(0);
    }
    
    void StereoTracker::updateID()
    {
        for (unsigned int i = 0; i < ids.size(); ++i)
            if (ids[i] == -1)
                ids[i] = n_id++;
    }
    
    void StereoTracker::undistortedPoints()
    {
        cur_left_un_pts.clear();
        cur_right_un_pts.clear();
        
        for (unsigned int i = 0; i < cur_left_pts.size(); ++i)
        {
            Eigen::Vector2d al(cur_left_pts[i].x, cur_left_pts[i].y);
            Eigen::Vector2d ar(cur_right_pts[i].x, cur_right_pts[i].y);
            
            Eigen::Vector3d bl, br;
            
            m_camera_left->liftProjective(al, bl);
            m_camera_right->liftProjective(ar, br);
            
            cur_left_un_pts.push_back(cv::Point2f(bl.x()/bl.z(), bl.y()/bl.z()));
            cur_right_un_pts.push_back(cv::Point2f(br.x()/br.z(), br.y()/br.z()));
        }
        
        
    }
    
    void StereoTracker::addPoints()
    {
        for (unsigned int i = 0; i < n_left_pts.size(); ++i)
        {
            forw_left_pts.push_back(n_left_pts[i]);
            forw_right_pts.push_back(n_right_pts[i]);
            ids.push_back(-1);
            track_cnt.push_back(1);
        }
    }
    
    void StereoTracker::setMask()
    {
        using namespace std;
        
        mask = cv::Mat(param.row, param.col, CV_8UC1, cv::Scalar(255));
        
        typedef pair<unsigned int, pair<cv::Point2f, pair<cv::Point2f, unsigned long long int>>> pts_pair;
        typedef vector<pts_pair> pts_pair_vector;
        
        pts_pair_vector cnt_lpts_rpts_id;
        
        for (unsigned int i = 0; i < forw_left_pts.size(); ++i)
            cnt_lpts_rpts_id.push_back(make_pair(track_cnt[i], make_pair(forw_left_pts[i], make_pair(forw_right_pts[i], ids[i]))));
        
        sort(cnt_lpts_rpts_id.begin(), cnt_lpts_rpts_id.end(), [](const pts_pair& a, const pts_pair& b){ return a.first > b.first; });
        
        forw_left_pts.clear();
        forw_right_pts.clear();
        ids.clear();
        track_cnt.clear();
        
        for (auto& it: cnt_lpts_rpts_id)
        {
            if (mask.at<uchar>(it.second.first) == 255)
            {
                forw_left_pts.push_back(it.second.first);
                forw_right_pts.push_back(it.second.second.first);
                track_cnt.push_back(it.first);
                ids.push_back(it.second.second.second);
                
                cv::circle(mask, it.second.first, param.min_dist, 0, -1);
            }
        }
    }
    
    void StereoTracker::rejectWithTwinRANSAC()
    {
        int origin_pts_num = forw_left_pts.size();
        
        if (forw_left_pts.size() >= 8)
        {
            std::vector<cv::Point2f> un_cur_left_pts(cur_left_pts.size()), un_forw_left_pts(forw_left_pts.size());
            
            double left_focal = 1.0/m_camera_left->getNormalPixel();
            
            for (unsigned int i = 0; i < cur_left_pts.size(); ++i)
            {
                Eigen::Vector3d tmp_p;
                m_camera_left->liftProjective(Eigen::Vector2d(cur_left_pts[i].x, cur_left_pts[i].y), tmp_p);
                
                tmp_p.x() = left_focal*tmp_p.x()/tmp_p.z() + param.col/2.0;
                tmp_p.y() = left_focal*tmp_p.y()/tmp_p.z() + param.row/2.0;
                un_cur_left_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
                
                m_camera_left->liftProjective(Eigen::Vector2d(forw_left_pts[i].x, forw_left_pts[i].y), tmp_p);
                tmp_p.x() = left_focal*tmp_p.x()/tmp_p.z() + param.col/2.0;
                tmp_p.y() = left_focal*tmp_p.y()/tmp_p.z() + param.row/2.0;
                un_forw_left_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            }
            
            std::vector<uchar> status;
            cv::findFundamentalMat(un_cur_left_pts, un_forw_left_pts, cv::FM_RANSAC, param.F_threshold, 0.99, status);
            
            reduceTwinVector<cv::Point2f>(prev_left_pts, prev_right_pts, status);
            reduceTwinVector<cv::Point2f>(cur_left_pts, cur_right_pts, status);
            reduceTwinVector<cv::Point2f>(forw_left_pts, forw_right_pts, status);
            reduceTwinVector<cv::Point2f>(cur_left_un_pts, cur_right_un_pts, status);
            // reduceTwinVector<cv::Point2f>(prev_left_un_pts, prev_right_un_pts, status);
            reduceVector<unsigned long long int>(ids, status);
            reduceVector<unsigned int>(track_cnt, status);
        }
        
        if (forw_right_pts.size() > 8)
        {
            std::vector<cv::Point2f> un_cur_right_pts(cur_right_pts.size()), un_forw_right_pts(forw_right_pts.size());
            
            double right_focal = 1.0/m_camera_right->getNormalPixel();
            
            for (unsigned int i = 0; i < cur_right_pts.size(); ++i)
            {
                Eigen::Vector3d tmp_p;
                m_camera_right->liftProjective(Eigen::Vector2d(cur_right_pts[i].x, cur_right_pts[i].y), tmp_p);
                
                tmp_p.x() = right_focal*tmp_p.x()/tmp_p.z() + param.col/2.0;
                tmp_p.y() = right_focal*tmp_p.y()/tmp_p.z() + param.row/2.0;
                un_cur_right_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
                
                m_camera_right->liftProjective(Eigen::Vector2d(forw_right_pts[i].x, forw_right_pts[i].y), tmp_p);
                tmp_p.x() = right_focal*tmp_p.x()/tmp_p.z() + param.col/2.0;
                tmp_p.y() = right_focal*tmp_p.y()/tmp_p.z() + param.row/2.0;
                un_forw_right_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            }
            
            std::vector<uchar> status;
            cv::findFundamentalMat(un_cur_right_pts, un_forw_right_pts, cv::FM_RANSAC, param.F_threshold, 0.99, status);
            
            reduceTwinVector<cv::Point2f>(prev_left_pts, prev_right_pts, status);
            reduceTwinVector<cv::Point2f>(cur_left_pts, cur_right_pts, status);
            reduceTwinVector<cv::Point2f>(forw_left_pts, forw_right_pts, status);
            reduceTwinVector<cv::Point2f>(cur_left_un_pts, cur_right_un_pts, status); // reduceTwinVector<cv::Point2f>(prev_left_un_pts, prev_right_un_pts, status);
            reduceVector<unsigned long long int>(ids, status);
            reduceVector<unsigned int>(track_cnt, status);            
        }
        
        int final_pts_num = forw_left_pts.size();
        
        if (origin_pts_num > 0 && final_pts_num != origin_pts_num && param.print_track_info)
            ROS_INFO("[Stereo Tracker]: Twin side RANSAC remove %f %% of points.", 100.0-100.0*float(final_pts_num)/float(origin_pts_num));
    }
    
    void StereoTracker::markStereoMatchOutlier(const std::vector<cv::Point2f>& left_pts, const std::vector<cv::Point2f>& right_pts, std::vector<uchar>& status)
    {
        Eigen::Isometry3d T_cl2cr = param.T_cr2i.inverse()*param.T_cl2i;
        
        Eigen::Matrix3d t_hat;
        const Eigen::Vector3d& t = T_cl2cr.translation();
        t_hat <<    0.0,   -t.z(),   t.y(),
                  t.z(),      0.0,  -t.x(),
                 -t.y(),    t.x(),     0.0;
        
        const Eigen::Matrix3d essen_mat = t_hat*T_cl2cr.linear();
        
        double unmatch_stereo_thres = 2.0*param.epipolar_thres/(1.0/m_camera_left->getNormalPixel() + 1.0/m_camera_right->getNormalPixel());
        
        int remove_num = 0;
        int total_num = 0;
        
        for (unsigned int i = 0; i < left_pts.size(); ++i)
        {
            if (status[i] == 0) continue;
            
            ++total_num;
            
            Eigen::Vector3d left_proj, right_proj;
            m_camera_left->liftProjective(Eigen::Vector2d(left_pts[i].x, left_pts[i].y), left_proj);
            m_camera_right->liftProjective(Eigen::Vector2d(right_pts[i].x, right_pts[i].y), right_proj);
            
            Eigen::Vector3d epipolar_line = essen_mat*left_proj;
            double norm_err = std::fabs((right_proj.transpose()*epipolar_line)[0]/std::sqrt(epipolar_line[0]*epipolar_line[0]+epipolar_line[1]*epipolar_line[1]));
            
            if (norm_err > unmatch_stereo_thres)
            {
                status[i] = 0;
                ++remove_num;
            }
        }
        
        if (total_num > 0 && param.print_track_info)
            ROS_INFO("[Stereo Tracker]: Remove %f %% of stereo matches through epipolar constrains!", float(remove_num)/float(total_num)*100.0);
        
    }
    
    bool StereoTracker::inBorder(const cv::Point2f& pt)
    {
        const int BORDER_SIZE = 1;
        int img_x = cvRound(pt.x);
        int img_y = cvRound(pt.y);
        
        return BORDER_SIZE <= img_x && img_x < param.col-BORDER_SIZE && BORDER_SIZE <= img_y && img_y < param.row-BORDER_SIZE;     
    }
}
