#include "IngvioFilter.h"

using namespace ingvio;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ingvio_estimator");
    ros::NodeHandle n("~");
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    IngvioFilterPtr ingvio_filter_ptr = std::make_shared<IngvioFilter>(n);
    
    ingvio_filter_ptr->initIO();
    
    ros::spin();
    
    return 0;
}
