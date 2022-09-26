#include "mono_tracker.h"

#include "Color.h"

namespace feature_tracker
{
    int MonoTracker::n_id = 0;
    
    void MonoTracker::init()
    {
        param.readParameters(nh);
        
        readIntrinsicParameter(param.cam_path);
        
        sub_img = nh.subscribe(param.image_topic, 100, &MonoTracker::img_callback, this);
        pub_mono_features = nh.advertise<feature_tracker::MonoFrame>("mono_feature", 100);
        pub_match = nh.advertise<sensor_msgs::Image>("mono_track", 100);
        
        first_image_flag = true;
        first_image_time = 0.0;
        last_image_time = 0.0;
        
        pub_this_frame = false;
        
        pub_count = 1;
    }
    
    void MonoTracker::img_callback(const sensor_msgs::ImageConstPtr& img_msg)
    {
        TicToc mono_timer;
        
        if (first_image_flag)
        {
            first_image_flag = false;
            first_image_time = img_msg->header.stamp.toSec();
            last_image_time = img_msg->header.stamp.toSec();
            return;
        }
        
        if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
        {
            ROS_WARN("[Mono Tracker]: Image discontinue! Mono tracker reset!");
            first_image_flag = true;
            first_image_time = 0.0;
            last_image_time = 0.0;
            pub_count = 1;
            pub_this_frame = false;
            return;
        }
        
        last_image_time = img_msg->header.stamp.toSec();
        
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
        
        
        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        
        
        readImage(ptr->image, img_msg->header.stamp.toSec());
        
        updateID();
        
        if (pub_this_frame)
        {
            ++pub_count;
            
            feature_tracker::MonoFrame::Ptr mono_frame_ptr(new feature_tracker::MonoFrame);
            mono_frame_ptr->header = img_msg->header;
            
            for (int i = 0; i < ids.size(); ++i)
            {
                if (track_cnt[i] > 1)
                {
                    feature_tracker::MonoMeas::Ptr mono_feature_ptr(new feature_tracker::MonoMeas);
                    
                    mono_feature_ptr->id = ids[i];
                    mono_feature_ptr->u0 = cur_un_pts[i].x;
                    mono_feature_ptr->v0 = cur_un_pts[i].y;
                    
                    mono_frame_ptr->mono_features.push_back(*mono_feature_ptr);
                }
            }
            
            if (mono_frame_ptr->mono_features.size() > 0)
                pub_mono_features.publish(mono_frame_ptr);
            
            if (param.show_track)
            {
                ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                
                cv::Mat tmp_img = ptr->image;
                
                for (unsigned int j = 0; j < cur_pts.size(); ++j)
                {
                    double len = std::min(1.0, 1.0*track_cnt[j]/param.window_size);
                    cv::circle(tmp_img, cur_pts[j], 2, cv::Scalar(255*(1-len), 0, 255*len), 2);
                }
                pub_match.publish(ptr->toImageMsg());
            }
        }
        
        if (param.show_timer)
        {
            double mono_time = mono_timer.toc();
            
            if (mono_time <= param.timer_warning_thres)
                std::cout << color::setBlue << "[MonoTracker]: Mono tracking processing time = " << mono_time << " (ms) " << color::resetColor << std::endl;
            else
                std::cout << color::setRed << "[MonoTracker]: Mono tracking processing time = " << mono_time << " (ms) " << color::resetColor << std::endl;
        }
        
    }
    
    void MonoTracker::updateID()
    {
        for (unsigned int i = 0; i < ids.size(); ++i)
            if (ids[i] == -1)
                ids[i] = n_id++;
    }
    
    
    void MonoTracker::readImage(const cv::Mat& _img, double _cur_time)
    {
        cv::Mat img;
        cur_time = _cur_time;
        
        if (param.equalize)
        {
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            clahe->apply(_img, img);
        }
        else
            img = _img;
            
        if (forw_img.empty())
            prev_img = cur_img = forw_img = img;
        else
            forw_img = img;
            
        forw_pts.clear();
        
        if (cur_pts.size() > 0)
        {
            std::vector<uchar> status;
            std::vector<float> err;
            
            cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
            
            for (int i = 0; i < int(forw_pts.size()); ++i)
                if (status[i] && !inBorder(forw_pts[i]))
                    status[i] = 0;
                
            reduceVector<cv::Point2f>(prev_pts, status);
            reduceVector<cv::Point2f>(cur_pts, status);
            reduceVector<cv::Point2f>(forw_pts, status);
            reduceVector<int>(ids, status);
            reduceVector<cv::Point2f>(cur_un_pts, status);
            reduceVector<int>(track_cnt, status);
        }
        
        for (auto& n: track_cnt)
            ++n;
        
        if (pub_this_frame)
        {
            rejectWithF();
            
            setMask();
            
            int n_max_cnt = param.max_cnt - static_cast<int>(forw_pts.size());
            
            if (n_max_cnt > 0)
            {
                if (mask.empty())
                    ROS_WARN("[Mono Tracker]: Mask is empty!");
                if (mask.type() != CV_8UC1)
                    ROS_WARN("[Mono Tracker]: Mask type wrong!");
                if (mask.size() != forw_img.size())
                    ROS_WARN("[Mono Tracker]: Mask size wrong!");
                
                cv::goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, param.min_dist, mask);
                
            }
            else
                n_pts.clear();
            
            addPoints();
        }
        
        prev_img = cur_img;
        prev_pts = cur_pts;
        prev_un_pts = cur_un_pts;
        
        cur_img = forw_img;
        cur_pts = forw_pts;
        
        undistortedPoints();
        
        prev_time = cur_time;
    }
    
    void MonoTracker::addPoints()
    {
        for (auto& p: n_pts)
        {
            forw_pts.push_back(p);
            ids.push_back(-1);
            track_cnt.push_back(1);
        }
    }   
    
    void MonoTracker::undistortedPoints()
    {
        cur_un_pts.clear();
        cur_un_pts_map.clear();
        
        for (unsigned int i = 0; i < cur_pts.size(); ++i)
        {
            Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            cur_un_pts.push_back(cv::Point2f(b.x()/b.z(), b.y()/b.z()));
            cur_un_pts_map.insert(std::make_pair(ids[i], cv::Point2f(b.x()/b.z(), b.y()/b.z())));
        }
        
        if (!prev_un_pts_map.empty())
        {
            double dt = cur_time - prev_time;
            pts_velocity.clear();
            
            for (unsigned int i = 0; i < cur_un_pts.size(); ++i)
            {
                if (ids[i] != -1)
                {
                    auto it = prev_un_pts_map.find(ids[i]);
                    if (it != prev_un_pts_map.end())
                    {
                        double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                        double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                        pts_velocity.push_back(cv::Point2f(v_x, v_y));
                    }
                    else
                        pts_velocity.push_back(cv::Point2f(0.0, 0.0));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0.0, 0.0));
            }
        }
        else
        {
            for (unsigned int i = 0; i < cur_pts.size(); ++i)
                pts_velocity.push_back(cv::Point2f(0.0, 0.0));
        }
        
        prev_un_pts_map = cur_un_pts_map;
    }
    
    void MonoTracker::setMask()
    {
        mask = cv::Mat(param.row, param.col, CV_8UC1, cv::Scalar(255));
        
        std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
        
        for (unsigned int i = 0; i < forw_pts.size(); ++i)
            cnt_pts_id.push_back(std::make_pair(track_cnt[i], std::make_pair(forw_pts[i], ids[i])));
        
        std::sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const std::pair<int, std::pair<cv::Point2f, int>>& a, const std::pair<int, std::pair<cv::Point2f, int>>& b){ return a.first > b.first; });
        
        forw_pts.clear();
        ids.clear();
        track_cnt.clear();
        
        for (auto& it: cnt_pts_id)
        {
            if (mask.at<uchar>(it.second.first) == 255)
            {
                forw_pts.push_back(it.second.first);
                ids.push_back(it.second.second);
                track_cnt.push_back(it.first);
                
                cv::circle(mask, it.second.first, param.min_dist, 0, -1);
            }
        }

    }
    
    
    void MonoTracker::rejectWithF()
    {
        if (forw_pts.size() >= 8)
        {
            std::vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
            
            for (unsigned int i = 0; i < cur_pts.size(); ++i)
            {
                Eigen::Vector3d tmp_p;
                m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
                
                tmp_p.x() = param.focal_length*tmp_p.x()/tmp_p.z() + param.col/2.0;
                tmp_p.y() = param.focal_length*tmp_p.y()/tmp_p.z() + param.row/2.0;
                un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
                
                m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
                tmp_p.x() = param.focal_length*tmp_p.x()/tmp_p.z() + param.col/2.0;
                tmp_p.y() = param.focal_length*tmp_p.y()/tmp_p.z() + param.row/2.0;
                un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            }
            
            std::vector<uchar> status;
            cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, param.f_threshold, 0.99, status);
            
            // int size_a = cur_pts.size();
            reduceVector<cv::Point2f>(prev_pts, status);
            reduceVector<cv::Point2f>(cur_pts, status);
            reduceVector<cv::Point2f>(forw_pts, status);
            reduceVector<cv::Point2f>(cur_un_pts, status);
            reduceVector<int>(ids, status);
            reduceVector<int>(track_cnt, status);
            
            // ROS_INFO("FM RANSAC: %d points reduced to -> %lu: %f", size_a, forw_pts.size(), 1.0*forw_pts.size()/size_a);
        }
    }
    
    
    bool MonoTracker::inBorder(const cv::Point2f& pt)
    {
        const int BORDER_SIZE = 1;
        int img_x = cvRound(pt.x);
        int img_y = cvRound(pt.y);
        
        return BORDER_SIZE <= img_x && img_x < param.col-BORDER_SIZE && BORDER_SIZE <= img_y && img_y < param.row-BORDER_SIZE;
    }
    
    void MonoTracker::readIntrinsicParameter(const std::string& calib_file)
    {
        ROS_INFO("[Mono Tracker]: Reading parameter of camera from: %s", calib_file.c_str());
        
        m_camera = camera_model::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    }
    
}
