#include "stereo_tracker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    feature_tracker::StereoTrackerPtr stereo_tracker_ptr(new feature_tracker::StereoTracker(n));
    
    stereo_tracker_ptr->init();
    
    ros::spin();
    return 0;
}
