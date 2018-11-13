#include <yumi_manager/yumi_manager.h>

class TestManager:public YumiManager
{
    public:
        TestManager(std::string point_action_topic, std::string home_action_topic, std::string pick_place_action_topic,std::string camera_topic, ros::NodeHandle* nh):YumiManager(point_action_topic,home_action_topic,pick_place_action_topic,camera_topic,nh)
        {

            ROS_INFO("Actions have been set");

        }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    std::string point_action_topic;
    std::string home_action_topic;
    std::string pick_place_action_topic;
    std::string camera_topic;
    local_nh.param<std::string>("home_action_topic", home_action_topic, "moveit_yumi_home");
    local_nh.param<std::string>("pick_place_action_topic", pick_place_action_topic, "moveit_yumi_pick_place");
    local_nh.param<std::string>("point_action_topic", point_action_topic, "moveit_yumi_point");
    local_nh.param<std::string>("camera_topic", camera_topic, "/kinect2/qhd/image_color");

    TestManager* testManager = new TestManager(point_action_topic,home_action_topic,pick_place_action_topic,camera_topic,&nh);

    //ros::spin();

     ros::Rate r(30);

    while(ros::ok())
    {

        ros::spinOnce();

    }

    delete testManager;

    return 0;
}



