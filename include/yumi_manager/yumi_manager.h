#include <ros/ros.h>


#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>
#include <boost/filesystem.hpp>

#include <cv_bridge/cv_bridge.h>

#include <perception_manager/QueryObjects.h>
#include <perception_manager/TabletopObject.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <yumi_actions/PickPlaceAction.h>
#include <yumi_actions/HomeAction.h>

#include <yumi_actions/PointAction.h>
#include <yumi_eneroth_bridge/Command.h>

#include <yumi_manager/SceneObjects.h>


#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <ros_cpp_utils/utils.h>

typedef actionlib::SimpleActionClient<yumi_actions::PickPlaceAction> PickPlaceClient;
typedef actionlib::SimpleActionClient<yumi_actions::PointAction> PointClient;
typedef actionlib::SimpleActionClient<yumi_actions::HomeAction> HomeClient;


class YumiManager
{
public:

    //YumiManager();

    virtual ~YumiManager();

    YumiManager(std::string point_action_topic,std::string home_action_topic, std::string pick_place_action_topic, std::string camera_topic, std::string controller_type, ros::NodeHandle* nh);

    virtual void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    static void callbackButtonPickPlaceAction(int state, void* userdata);

    static void callbackButtonPointAction(int state, void* userdata);

    static void callbackButtonHomeAction(int state, void* userdata);

    static void callbackButtonPlanAction(int state, void* userdata);

    static void callbackButtonRefreshScene(int state, void* userdata);

    static void callbackWorkspace(int event, int x, int y, int flags, void* userdata);

    static void callbackButtonShowWorkspace(int state, void* userdata);

    

protected:

    std::string home_action_topic;
    std::string point_action_topic;
    std::string pick_place_action_topic;

    std::string camera_topic;

    PickPlaceClient* pick_place_client;
    HomeClient* home_client;
    PointClient* point_client;

    std::vector<perception_manager::TabletopObject> objects;

    bool image_resized;
    int selected_index;
    bool pick_place_action;
    bool point_action;
    bool home_action;

    bool yumi_busy;

    bool plan_action;

    ros::NodeHandle* nh;

    int workspace_min_x;

    int workspace_min_y;

    int workspace_max_y;

    int workspace_max_x;

    std::string controller_type;
    
    void publishScene(int yumi_status, ros::Publisher* publisher);


private:
    image_transport::ImageTransport *it;
    image_transport::Subscriber* it_sub;

    bool readWorkspaceConfig(std::string path, int *minX, int *maxX, int *minY, int *maxY);


    void drawWorkspace(cv::Mat* img);

    bool draw_workspace;











};
