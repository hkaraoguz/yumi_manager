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

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <yumi_demos/PlanforAction.h>

#include <tf/transform_listener.h>

using namespace std;
using namespace cv;

bool point = false;
bool pick_and_place = true;
bool home = false;
bool planfor_action = false;

bool yumi_busy = false;

int selected_index = -1;

typedef actionlib::SimpleActionClient<yumi_actions::PickPlaceAction> pickplaceClient;
typedef actionlib::SimpleActionClient<yumi_actions::PointAction> pointClient;
typedef actionlib::SimpleActionClient<yumi_actions::HomeAction> homeClient;

//Mat temp_img;
Mat ss_img;


tf::TransformListener* listener;

double camera_fx = 1049.46;
double camera_fy = 1050.24;

int camera_principal_center_x=951;
int camera_principal_center_y=534;


int leap_pixel_x;
int leap_pixel_y;


yumi_actions::PickPlaceGoal pickplace_goal;
yumi_actions::PointGoal point_goal;


vector<perception_manager::TabletopObject> objects;

ros::ServiceClient query_objects_client;

ros::ServiceClient planforaction_client;

ros::Publisher scene_publisher;



string getHomePath()
{
    uid_t uid = getuid();
    struct passwd *pw = getpwuid(uid);

    if (pw == NULL) {
        ROS_ERROR("Failed to get homedir. Cannot save configuration file\n");
        return "";
    }

    // printf("%s\n", pw->pw_dir);
    string str(pw->pw_dir);
    return str;

}
bool saveImage()
{
    string configpath = getHomePath();

    configpath += "/yumi_manager/";

    boost::filesystem::path dir(configpath);

    if(!(boost::filesystem::exists(dir)))
    {
        std::cout<<"Doesn't Exists"<<std::endl;

    }

    if (boost::filesystem::create_directory(dir))
        std::cout << "....Successfully Created !" << std::endl;

    stringstream ss;

    ss<<ros::Time::now();

    configpath += "image_";
    configpath += ss.str();
    configpath+= ".jpg";

    if(ss_img.rows > 0)
    {
        cv::imwrite(configpath.data(),ss_img);
        return true;
    }

    return false;
}

void doneCbPickPlace(const actionlib::SimpleClientGoalState& state,
                     const yumi_actions::PickPlaceResultConstPtr& result)
{
    yumi_busy = false;
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %d", result->result);

    objects.clear();
    perception_manager::QueryObjects query_objects_srv;

    if(query_objects_client.call(query_objects_srv))
    {
        objects = query_objects_srv.response.objects ;

    }

    yumi_manager::SceneObjects so;

    if(result->result == -2)
    {
        so.yumi_status=2;
        scene_publisher.publish(so);
        return;

    }

    /* if(saveImage())
    {
        ROS_INFO("Image successfully saved!");
    }*/

    so.array = objects;
    so.yumi_status=0;
    scene_publisher.publish(so);
    //ros::shutdown();
}
void doneCbPoint(const actionlib::SimpleClientGoalState& state,
                 const yumi_actions::PointResultConstPtr& result)
{
    yumi_busy = false;
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %d", result->result);

    objects.clear();
    perception_manager::QueryObjects query_objects_srv;


    yumi_manager::SceneObjects so;
    // This means task is aborted because of planning fail
    if(result->result == -2)
    {
        so.yumi_status=2;
        scene_publisher.publish(so);
        return;

    }

    /* if(query_objects_client.call(query_objects_srv))
    {
        objects = query_objects_srv.response.objects ;

    }


    so.array = objects;
    so.yumi_status=0;
    scene_publisher.publish(so);*/
    //ros::shutdown();
}
void doneCbHome(const actionlib::SimpleClientGoalState& state,
                const yumi_actions::HomeResultConstPtr& result)
{
    yumi_busy = false;
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %d", result->result);


    perception_manager::QueryObjects query_objects_srv;

    if(query_objects_client.call(query_objects_srv))
    {
        objects = query_objects_srv.response.objects ;

    }

    yumi_manager::SceneObjects so;

    if(result->result == -2)
    {
        so.yumi_status=2;
        scene_publisher.publish(so);
        return;

    }

    if(saveImage())
    {
        ROS_INFO("Image successfully saved!");
    }

    so.array = objects;
    so.yumi_status=0;
    scene_publisher.publish(so);
    //ros::shutdown();
}

void leapHandCallback(std_msgs::Float32 msg)
{
    if(msg.data == 1.0)
    {
        int min_sum = 100000;
        int min_index = -1;
        for(size_t i =0 ; i < objects.size(); i++)
        {
            int sum = 0;

            sum+= abs(objects[i].pixelposcenterx-leap_pixel_x);
            sum+= abs(leap_pixel_y-objects[i].pixelposcentery);

            if(sum < min_sum)
            {
                min_sum = sum;
                min_index = i;
            }
        }

        if(min_index >= 0  && min_sum <= 10)
        {
            ROS_INFO("Selected id: %d",min_index);

            selected_index = min_index;

            if(pick_and_place)
            {

                pickplace_goal.location.position.x = objects[min_index].metricposcenterx;
                pickplace_goal.location.position.y = objects[min_index].metricposcentery;

                if(objects[min_index].angle > 0)
                    pickplace_goal.location.orientation.z = objects[min_index].angle;
                else
                    pickplace_goal.location.orientation.z = objects[min_index].angle;
                //wait for the action to return
                // bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
                ROS_INFO("Angle %.2f", pickplace_goal.location.orientation.z*180/3.14159);

            }
        }
    }

}


// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
    yumi_busy = true;

}

// Called every time feedback is received for the goal
void feedbackCbPickPlace(const yumi_actions::PickPlaceFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback of length");
}
// Called every time feedback is received for the goal
void feedbackCbPoint(const yumi_actions::PointFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback of length");
}
// Called every time feedback is received for the goal
void feedbackCbHome(const yumi_actions::HomeFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback of length");
}

/*void drawWorkspace(Mat img)yumi_base_link
{

    Rect rect(workspace_min_x,workspace_min_y,workspace_max_x-workspace_min_x,workspace_max_y-workspace_min_y);

    rectangle(img,rect,cv::Scalar(0,0,255),2);

}*/


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        int min_sum = 100000;
        int min_index = -1;
        for(size_t i =0 ; i < objects.size(); i++)
        {
            int sum = 0;

            sum+= abs(objects[i].pixelposcenterx-x*2);
            sum+= abs(y*2-objects[i].pixelposcentery);

            if(sum < min_sum)
            {
                min_sum = sum;
                min_index = i;
            }
        }

        if(min_index >= 0  && min_sum <= 10)
        {
            ROS_INFO("Selected id: %d",min_index);

            selected_index = min_index;

            if(pick_and_place)
            {

                pickplace_goal.location.position.x = objects[min_index].metricposcenterx;
                pickplace_goal.location.position.y = objects[min_index].metricposcentery;

                if(objects[min_index].angle > 0)
                    pickplace_goal.location.orientation.z = objects[min_index].angle;
                else
                    pickplace_goal.location.orientation.z = objects[min_index].angle;
                //wait for the action to return
                // bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
                ROS_INFO("Angle %.2f", pickplace_goal.location.orientation.z*180/3.14159);

            }
            else if(point)
            {
                point_goal.location.position.x = objects[min_index].metricposcenterx;
                point_goal.location.position.y = objects[min_index].metricposcentery;


            }




        }


    }
    /* else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

    }*/
}

void calculatePixelPosition(double posx, double posy, double posz, int* pixelx, int* pixely)
{
    *pixelx = (int)(posx*camera_fx/posz)+camera_principal_center_x;

    *pixely= (int)(posy*camera_fy/posz)+camera_principal_center_y;

}



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    char key = (char)cv::waitKey(50);


    if( key == 27){


        ROS_INFO("Shutting down the node...");
        ros::shutdown();
    }



    tf::StampedTransform transform1;

    int pixelx=0,pixely=0;

    try{
        //if(listener.waitForTransform("yumi_base_link","teleop_left_frame",ros::Time::now(), ros::Duration(1.0)))
        //{
        listener->lookupTransform("/kinect2_rgb_optical_frame","/teleop_left_frame",
                                  ros::Time(0), transform1);

      //  std::cout<<"Left hand position"<<transform1.getOrigin().getX()<<" "<<transform1.getOrigin().getZ()<<std::endl;



        calculatePixelPosition(transform1.getOrigin().getX(),transform1.getOrigin().getY(),transform1.getOrigin().getZ(),&leap_pixel_x,&leap_pixel_y);

     //   std::cout<<"Left hand position in pixels "<<leap_pixel_x<<" "<<leap_pixel_y<<std::endl;

        //}



    }

    catch (tf::TransformException ex){
       // ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }

    Mat temp_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    ss_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    for(size_t i =0 ; i < objects.size(); i++)
    {
        Point pt;
        pt.x = objects[i].pixelposcenterx;
        pt.y = objects[i].pixelposcentery;

        circle(temp_img, pt, 3, cv::Scalar(0,0,255), -1);
        stringstream ss;
        ss<<objects[i].id;
        pt.x +=3;
        pt.x +=3;
        putText(temp_img, ss.str(), pt, 0, 0.4, cv::Scalar(0,0,255), 2);
    }

    Point pt;

    pt.x = leap_pixel_x;
    pt.y = leap_pixel_y;

    circle(temp_img, pt, 6, cv::Scalar(255,0,255), -1);

    Mat img;
    resize(temp_img,img,cv::Size(0,0),0.5,0.5);
    imshow("Workspace",img);
}
bool is_ok = false;
void callBackButtonPickandPlace(int state, void*);
void callBackButtonPoint(int state, void*)
{
    if(state == 1)
    {
        pick_and_place = false;
        point = true;

        ROS_INFO("Point action is selected");
    }

}
void callBackButtonHome(int state, void*)
{
    home = true;
    selected_index = -1;
    //pick_and_place = false;
    //point = false;
    //yumi_actions::HomeGoal hgoal;
    ROS_INFO("Sending goal to return home position action.");


}
void callBackButtonPlanAction(int state, void*)
{

    if (planfor_action){

        ROS_WARN("Plan for action disabled!");

        planfor_action = false;

    }
    else{
        planfor_action = true;

        ROS_WARN("Plan for action enabled!");
    }


}
void callBackButtonRefreshScene(int state, void*)
{
    objects.clear();
    perception_manager::QueryObjects query_objects_srv;

    if(saveImage())
    {
        ROS_INFO("Image successfully saved!");
    }

    if(query_objects_client.call(query_objects_srv))
    {
        objects = query_objects_srv.response.objects ;

        yumi_manager::SceneObjects so;
        so.array = objects;
        scene_publisher.publish(so);

    }
}

void commandCallback(const yumi_eneroth_bridge::CommandConstPtr& msg)
{
    if (yumi_busy)
    {
        yumi_manager::SceneObjects so;
        so.yumi_status = 1;
        scene_publisher.publish(so);
    }
    if (msg->type == "home")
    {
        ROS_INFO("Home action ");
        selected_index = -1;
        home = true;
        // pick_and_place = false;
        // point = false;
        return;
    }

    float min_sum = 1000.0;
    int min_index = -1;

    for(size_t i =0 ; i < objects.size(); i++)
    {
        float sum = 0;

        //cout<<objects[i].metricpostablecenterx<<" "<<objects[i].metricpostablecentery<<endl;
        //cout<<msg->posx<<" "<<msg->posy<<endl;

        sum+= fabs(objects[i].metricposcenterx - msg->posx);
        sum+= fabs(objects[i].metricposcentery - msg->posy);

        if(sum < min_sum)
        {
            min_sum = sum;
            min_index = i;
        }

        // std::cout<<min_index<<endl;
    }

    if(min_index >= 0  && min_sum <= 0.05)
    {
        selected_index = min_index;

        if (msg->type == "point")
        {
            ROS_INFO("Point action ");

            pick_and_place = false;
            point = true;

            point_goal.location.position.x = objects[min_index].metricposcenterx;
            point_goal.location.position.y = objects[min_index].metricposcentery;
        }

        else if (msg->type == "pick")
        {
            ROS_INFO("Pick and Place action ");

            pick_and_place = true;
            point = false;

            pickplace_goal.location.position.x = objects[min_index].metricposcenterx;
            pickplace_goal.location.position.y = objects[min_index].metricposcentery;

            if(objects[min_index].angle > 0)
                pickplace_goal.location.orientation.z = objects[min_index].angle;
            else
                pickplace_goal.location.orientation.z = objects[min_index].angle;
            //wait for the action to return
            // bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
            ROS_INFO("Angle %.2f", pickplace_goal.location.orientation.z*180/3.14159);
        }


    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leap_manager_node");
    ros::NodeHandle nh;


    //Create a window
    namedWindow("Workspace", 1);

    //set the callback function for any mouse event
    setMouseCallback("Workspace", CallBackFunc, NULL);

    createButton("pick_and_place",callBackButtonPickandPlace,NULL,CV_RADIOBOX,1);
    createButton("point",callBackButtonPoint,NULL,CV_RADIOBOX,0);
    createButton("Refresh Scene",callBackButtonRefreshScene,NULL,CV_PUSH_BUTTON);
    createButton("Home Position",callBackButtonHome,NULL,CV_PUSH_BUTTON);
    createButton("Plan Action",callBackButtonPlanAction,NULL,CV_PUSH_BUTTON);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("kinect2/hd/image_color", 1, imageCallback);

    listener = new tf::TransformListener(nh);

    ros::Subscriber subs = nh.subscribe("yumi_eneroth_bridge/command",1,commandCallback);

    ros::Subscriber leap_hand_subs = nh.subscribe("leap_hands/grasp_left",1,leapHandCallback);

    ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("yumi/traj_moveit",10);

    scene_publisher = nh.advertise<yumi_manager::SceneObjects>("yumi_manager/scene_objects",1);

    query_objects_client = nh.serviceClient<perception_manager::QueryObjects>("perception_manager/query_objects");

    planforaction_client = nh.serviceClient<yumi_demos::PlanforAction>("yumi_plan_action");

    perception_manager::QueryObjects query_objects_srv;

    if(query_objects_client.call(query_objects_srv))
    {
        objects = query_objects_srv.response.objects ;

    }

    yumi_manager::SceneObjects so;
    so.yumi_status=0;
    so.array = objects;
    scene_publisher.publish(so);

    pickplaceClient ppc("skill_yumi_pick_and_place", true);
    pointClient pc("skill_yumi_point", true);
    homeClient hc("skill_yumi_home", true);


    ppc.waitForServer(ros::Duration(2));
    pc.waitForServer(ros::Duration(2));
    hc.waitForServer(ros::Duration(2));

    ROS_INFO("Action server started, waiting for goal.");



    //show the image
    // imshow("My Window", img);



    //  ros::ServiceServer workspace_service = nh.advertiseService("workspace_segmentation/GetWorkspace",findWorkspace);
    ros::Rate r(30);
    while(ros::ok())
    {

        ros::spinOnce();


        if(selected_index >= 0)
        {

            selected_index = -1;

            if(pick_and_place && !yumi_busy && !planfor_action)
            {
                // send a goal to the action
                ROS_INFO("Sending goal to pick and place action.");
                ppc.sendGoal(pickplace_goal,&doneCbPickPlace, &activeCb, &feedbackCbPickPlace);
            }
            else if(point && !yumi_busy&& !planfor_action)
            {
                ROS_INFO("Sending goal to point action.");
                pc.sendGoal(point_goal,&doneCbPoint,&activeCb,&feedbackCbPoint);

            }
            else if(pick_and_place && planfor_action)
            {


                yumi_demos::PlanforAction planfor_action_srv;

                planfor_action_srv.request.action = 0;

                planfor_action_srv.request.location = pickplace_goal.location;

                if(planforaction_client.call(planfor_action_srv))
                {
                    for(auto robottraj:planfor_action_srv.response.trajectories) {

                        joint_pub.publish(robottraj.joint_trajectory);
                    }

                }


            }
            else if(point && planfor_action)
            {


                yumi_demos::PlanforAction planfor_action_srv;

                planfor_action_srv.request.action = 1;

                planfor_action_srv.request.location = point_goal.location;

                if(planforaction_client.call(planfor_action_srv))
                {
                    for(auto robottraj:planfor_action_srv.response.trajectories) {

                        joint_pub.publish(robottraj.joint_trajectory);
                    }

                }


            }
        }

        if(home && !yumi_busy)
        {
            selected_index = -1;
            home = false;
            yumi_actions::HomeGoal hgoal;
            ROS_INFO("Sending goal to return home position action.");
            hc.sendGoal(hgoal,&doneCbHome,&activeCb,&feedbackCbHome);

        }



        r.sleep();

    }
}

void callBackButtonPickandPlace(int state, void*)
{
    if(state == 1)
    {

        pick_and_place = true;
        point = false;
        ROS_INFO("Pick and place action is selected");
    }
}


