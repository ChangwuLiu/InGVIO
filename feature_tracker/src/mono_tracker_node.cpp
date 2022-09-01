#include "mono_tracker.h"
    
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mono_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    feature_tracker::MonoTrackerPtr mono_tracker_ptr(new feature_tracker::MonoTracker(n));
    
    mono_tracker_ptr->init();
        
    ros::spin();
    return 0;
}
    
