#include <yumi_manager/yumi_manager.h>

#include <yumi_actions/PlanPickPlace.h>

#include <yumi_actions/PlanPoint.h>

#include <yumi_manager/SceneObjects.h>

#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>

using namespace std;

using namespace cv;


class LeapManager:public YumiManager
{

private:

    // These are the goal objects for actions

    yumi_actions::PickPlaceActionGoal pick_place_goal;

    yumi_actions::HomeActionGoal home_goal;

    yumi_actions::PointActionGoal point_goal;

    ros::Publisher trajectory_publisher ;

    yumi_manager::SceneObjects so;

    ros::Publisher scene_publisher;

    std::string camera_info_topic;

    ros::Subscriber camera_info_subscriber;

    float camera_focal_length_x ;

    float camera_focal_length_y;

    float camera_center_x;

    float camera_center_y;

    ros::Subscriber leap_hand_subs;

    int leap_pixel_x;

    int leap_pixel_y;

    std::string camera_optical_frame;

    std::string leapmotion_frame;

    tf::TransformListener* listener;



public:


    LeapManager(std::string point_action_topic, std::string home_action_topic, std::string pick_place_action_topic,std::string camera_topic,std::string camera_info_topic, std::string controller_type, std::string camera_optical_frame,std::string leapmotion_frame, ros::NodeHandle* nh):YumiManager(point_action_topic,home_action_topic,pick_place_action_topic,camera_topic,controller_type,nh)
    {

        ROS_INFO("Action subscribers have been set. Manager ready...");

        trajectory_publisher = this->nh->advertise<trajectory_msgs::JointTrajectory>("/yumi_manager/moveit_trajectory",10);

        // This publishes the current state of the scene and latest status of yumi
        this->scene_publisher = nh->advertise<yumi_manager::SceneObjects>("yumi_manager/scene_objects",1);

        this->camera_info_topic = camera_info_topic;

        this->camera_info_subscriber = nh->subscribe(this->camera_info_topic,1,&LeapManager::cameraInfoCallback,this);

        this->leap_hand_subs = nh->subscribe("leap_hands/grasp_left",1,&LeapManager::leapHandCallback,this);

        this->camera_optical_frame = camera_optical_frame;

        this->leapmotion_frame = leapmotion_frame;

        this->listener = new tf::TransformListener(*nh);


    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {

        char key = (char)cv::waitKey(20);


        if( key == 27){


            ROS_INFO("Shutting down the node...");
            ros::shutdown();
        }



        tf::StampedTransform transform;



        try
        {
            //if(listener.waitForTransform("yumi_base_link","teleop_left_frame",ros::Time::now(), ros::Duration(1.0)))
            //{
            if(listener->waitForTransform(this->camera_optical_frame,this->leapmotion_frame,ros::Time::now(), ros::Duration(1.0))){
                listener->lookupTransform(this->camera_optical_frame,this->leapmotion_frame,
                                          ros::Time(0), transform);

                //  std::cout<<"Left hand position"<<transform1.getOrigin().getX()<<" "<<transform1.getOrigin().getZ()<<std::endl;



                calculatePixelPosition(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ(),&leap_pixel_x,&leap_pixel_y);
            }

            //   std::cout<<"Left hand position in pixels "<<leap_pixel_x<<" "<<leap_pixel_y<<std::endl;

            //}



        }

        catch (tf::TransformException ex){
            //ROS_ERROR("%s",ex.what());
            //ros::Duration(1.0).sleep();
        }

        Mat temp_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat ss_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

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

        if (temp_img.rows > 1000)
        {
            this->image_resized = true;
            Mat img;
            resize(temp_img,img,cv::Size(0,0),0.5,0.5);
            imshow("Workspace",img);

        }
        else
        {
            imshow("Workspace",temp_img);

        }

        cv::waitKey(1);
    }

    void calculatePixelPosition(double posx, double posy, double posz, int* pixelx, int* pixely)
    {
        *pixelx = (int)(posx*this->camera_focal_length_x/posz)+this->camera_center_x;

        *pixely= (int)(posy*this->camera_focal_length_y/posz)+this->camera_center_y;

    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
    {
        this->camera_focal_length_x = msg.K[0];
        this->camera_focal_length_y = msg.K[4];

        this->camera_center_x = msg.K[2];
        this->camera_center_y = msg.K[5];

        this->camera_info_subscriber.shutdown();


    }
    // User's left hand is closed
    void leapHandCallback(const std_msgs::Float32& msg)
    {
        if(msg.data == 1.0)
        {
            int min_sum = 100000;
            int min_index = -1;

            for(size_t i =0 ; i < this->objects.size(); i++)
            {
                int sum = 0;

                sum+= abs(this->objects[i].pixelposcenterx-leap_pixel_x);
                sum+= abs(leap_pixel_y-this->objects[i].pixelposcentery);

                if(sum < min_sum)
                {
                    min_sum = sum;
                    min_index = i;
                }
            }

            if(min_index >= 0  && min_sum <= 30)
            {
                ROS_INFO("Selected id: %d",min_index);

                selected_index = min_index;

                /*if(pick_and_place)
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

                }*/
            }
        }

    }


    void doneCbPickPlace(const actionlib::SimpleClientGoalState& state,
                         const yumi_actions::PickPlaceResultConstPtr& result)
    {
        this->yumi_busy = false;

        ROS_INFO("Finished in state [%s]", state.toString().c_str());

        ROS_INFO("Answer: %d", result->result);

        this->objects.clear();

        ros::ServiceClient query_objects_client = this->nh->serviceClient<perception_manager::QueryObjects>("perception_manager/query_objects");


        perception_manager::QueryObjects query_objects_srv;

        if(query_objects_client.call(query_objects_srv))
        {
            this->objects = query_objects_srv.response.objects ;

        }


        this->publishScene(result->result,&this->scene_publisher);


    }
    void doneCbPoint(const actionlib::SimpleClientGoalState& state,
                     const yumi_actions::PointResultConstPtr& result)
    {
        this->yumi_busy = false;
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Answer: %d", result->result);

        this->objects.clear();


        ros::ServiceClient query_objects_client = this->nh->serviceClient<perception_manager::QueryObjects>("perception_manager/query_objects");


        perception_manager::QueryObjects query_objects_srv;

        if(query_objects_client.call(query_objects_srv))
        {
            objects = query_objects_srv.response.objects ;

        }


        this->publishScene(result->result,&this->scene_publisher);

    }
    void doneCbHome(const actionlib::SimpleClientGoalState& state,
                    const yumi_actions::HomeResultConstPtr& result)
    {
        this->yumi_busy = false;
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Answer: %d", result->result);

        this->objects.clear();

        ros::ServiceClient query_objects_client = this->nh->serviceClient<perception_manager::QueryObjects>("perception_manager/query_objects");


        perception_manager::QueryObjects query_objects_srv;

        if(query_objects_client.call(query_objects_srv))
        {
            objects = query_objects_srv.response.objects ;

        }


        this->publishScene(result->result,&this->scene_publisher);


    }


    // Called once when the goal becomes active
    void activeCb()
    {
        ROS_INFO("Goal just went active");
        this->yumi_busy = true;

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


    void loop()
    {
        // If yumi is available
        if(!yumi_busy)
        {

            if(this->home_action)
            {
                this->selected_index = -1;

                this->home_action = false;

                //this->home_client->sendGoal(home_goal.goal);

                this->home_client->sendGoal(home_goal.goal,boost::bind(&LeapManager::doneCbHome, this, _1, _2),boost::bind(&LeapManager::activeCb, this),boost::bind(&LeapManager::feedbackCbHome, this, _1));


            }

            // If an object is selected
            else if(this->selected_index >= 0)
            {
                //If pick and place is selected
                if(this->pick_place_action )
                {



                    this->pick_place_goal.goal.location.position.x = this->objects[this->selected_index].metricposcenterx;
                    this->pick_place_goal.goal.location.position.y = this->objects[this->selected_index].metricposcentery;
                    this->pick_place_goal.goal.location.position.z = this->objects[this->selected_index].metricposcenterz;

                    this->pick_place_goal.goal.location.orientation.z = this->objects[this->selected_index].angle;

                    std::cout<<"Pick Place Goal "<<this->objects[this->selected_index].metricposcenterx<<" "<<this->objects[this->selected_index].metricposcentery<<" "<<this->objects[this->selected_index].metricposcenterz<<std::endl;

                    if(this->plan_action && controller_type=="moveit")
                    {

                        yumi_actions::PlanPickPlace plan_action_srv;

                        plan_action_srv.request.goal = this->pick_place_goal.goal.location ;

                        //scene_publisher = nh.advertise<yumi_manager::SceneObjects>("yumi_manager/scene_objects",1);

                        //query_objects_client = this->nh->serviceClient<perception_manager::QueryObjects>("perception_manager/query_objects");

                        ros::ServiceClient planaction_client = this->nh->serviceClient<yumi_actions::PlanPickPlace>("moveit_yumi_plan_pick_place_action");


                        if(planaction_client.call(plan_action_srv))
                        {
                            for(auto robottraj:plan_action_srv.response.trajectories) {

                                trajectory_publisher.publish(robottraj.joint_trajectory);
                            }

                        }

                        this->selected_index = -1;

                        return;

                    }
                    else if (this->plan_action && controller_type != "moveit")
                    {
                        ROS_WARN("Action cannot be planned ahead for controllers other than MoveIt");
                    }

                    this->pick_place_client->sendGoal(this->pick_place_goal.goal,boost::bind(&LeapManager::doneCbPickPlace, this, _1, _2),boost::bind(&LeapManager::activeCb, this),boost::bind(&LeapManager::feedbackCbPickPlace, this, _1));

                    this->selected_index = -1;

                }
                else if(this->point_action)
                {
                    this->point_goal.goal.location.position.x = this->objects[this->selected_index].metricposcenterx;
                    this->point_goal.goal.location.position.y = this->objects[this->selected_index].metricposcentery;
                    this->point_goal.goal.location.position.z = this->objects[this->selected_index].metricposcenterz;

                    this->point_goal.goal.location.orientation.z = this->objects[this->selected_index].angle;

                    std::cout<<" Point Goal "<<this->objects[this->selected_index].metricposcenterx<<" "<<this->objects[this->selected_index].metricposcentery<<" "<<this->objects[this->selected_index].metricposcenterz<<std::endl;


                    if(this->plan_action && this->controller_type=="moveit")
                    {

                        yumi_actions::PlanPoint plan_point_srv;

                        plan_point_srv.request.goal = this->point_goal.goal.location ;

                        //scene_publisher = nh.advertise<yumi_manager::SceneObjects>("yumi_manager/scene_objects",1);

                        //query_objects_client = this->nh->serviceClient<perception_manager::QueryObjects>("perception_manager/query_objects");

                        ros::ServiceClient plan_action_client = this->nh->serviceClient<yumi_actions::PlanPoint>("moveit_yumi_plan_point_action");


                        if(plan_action_client.call(plan_point_srv))
                        {
                            for(auto robottraj:plan_point_srv.response.trajectories) {

                                trajectory_publisher.publish(robottraj.joint_trajectory);
                            }

                        }

                        this->selected_index = -1;

                        return;

                    }
                    else if (this->plan_action && controller_type != "moveit")
                    {
                        ROS_WARN("Action cannot be planned ahead for controllers other than MoveIt");
                    }





                    this->point_client->sendGoal(this->point_goal.goal,boost::bind(&LeapManager::doneCbPoint, this, _1, _2),boost::bind(&LeapManager::activeCb, this),boost::bind(&LeapManager::feedbackCbPoint, this, _1));

                    this->selected_index = -1;

                }

            }
        }

    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "leap_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    std::string point_action_topic;
    std::string home_action_topic;
    std::string pick_place_action_topic;
    std::string camera_topic;
    std::string controller_type;
    std::string camera_info_topic;

    std::string camera_optical_frame;
    std::string leapmotion_frame;


    local_nh.param<std::string>("controller_type", controller_type, "moveit");
    local_nh.param<std::string>("camera_topic", camera_topic, "/kinect2/qhd/image_color");
    local_nh.param<std::string>("camera_info_topic",camera_info_topic,"/kinect2/qhd/camera_info");

    local_nh.param<std::string>("camera_optical_frame",camera_optical_frame,"/kinect2_rgb_optical_frame");

    local_nh.param<std::string>("leapmotion_frame",leapmotion_frame,"/teleop_left_frame");





    home_action_topic = controller_type+"_yumi_home";
    point_action_topic = controller_type+"_yumi_point";
    pick_place_action_topic = controller_type+"_yumi_pick_place";


    ROS_INFO("Controller type is %s",controller_type.data());

    LeapManager* leapManager = new LeapManager(point_action_topic,home_action_topic,pick_place_action_topic,camera_topic,camera_info_topic,controller_type,camera_optical_frame,leapmotion_frame,&nh);

    ros::Rate r(30);

    while(ros::ok())
    {

        ros::spinOnce();

        leapManager->loop();

    }

    delete leapManager;

    return 0;
}




