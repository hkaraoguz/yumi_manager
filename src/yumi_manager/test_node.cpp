#include <yumi_manager/yumi_manager.h>

#include <yumi_actions/PlanPickPlace.h>

#include <yumi_actions/PlanPoint.h>

#include <yumi_manager/SceneObjects.h>


class TestManager:public YumiManager
{

private:

    // These are the goal objects for actions

    yumi_actions::PickPlaceActionGoal pick_place_goal;

    yumi_actions::HomeActionGoal home_goal;

    yumi_actions::PointActionGoal point_goal;

    ros::Publisher trajectory_publisher ;

    yumi_manager::SceneObjects so;

    ros::Publisher scene_publisher;



public:


    TestManager(std::string point_action_topic, std::string home_action_topic, std::string pick_place_action_topic,std::string camera_topic, std::string controller_type, ros::NodeHandle* nh):YumiManager(point_action_topic,home_action_topic,pick_place_action_topic,camera_topic,controller_type,nh)
    {

        ROS_INFO("Action subscribers have been set. Manager ready...");

        trajectory_publisher = this->nh->advertise<trajectory_msgs::JointTrajectory>("/yumi_manager/moveit_trajectory",10);

        // This publishes the current state of the scene and latest status of yumi
        this->scene_publisher = nh->advertise<yumi_manager::SceneObjects>("yumi_manager/scene_objects",1);


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

                this->home_client->sendGoal(home_goal.goal,boost::bind(&TestManager::doneCbHome, this, _1, _2),boost::bind(&TestManager::activeCb, this),boost::bind(&TestManager::feedbackCbHome, this, _1));


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

                    this->pick_place_client->sendGoal(this->pick_place_goal.goal,boost::bind(&TestManager::doneCbPickPlace, this, _1, _2),boost::bind(&TestManager::activeCb, this),boost::bind(&TestManager::feedbackCbPickPlace, this, _1));

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





                    this->point_client->sendGoal(this->point_goal.goal,boost::bind(&TestManager::doneCbPoint, this, _1, _2),boost::bind(&TestManager::activeCb, this),boost::bind(&TestManager::feedbackCbPoint, this, _1));

                    this->selected_index = -1;

                }

            }
        }

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
    std::string controller_type;


    local_nh.param<std::string>("controller_type", controller_type, "moveit");
    local_nh.param<std::string>("camera_topic", camera_topic, "/kinect2/qhd/image_color");


    home_action_topic = controller_type+"_yumi_home";
    point_action_topic = controller_type+"_yumi_point";
    pick_place_action_topic = controller_type+"_yumi_pick_place";


    ROS_INFO("Controller type is %s",controller_type.data());

    TestManager* testManager = new TestManager(point_action_topic,home_action_topic,pick_place_action_topic,camera_topic,controller_type,&nh);

    ros::Rate r(30);

    while(ros::ok())
    {

        ros::spinOnce();

        testManager->loop();

    }

    delete testManager;

    return 0;
}



