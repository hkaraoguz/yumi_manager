#include <yumi_manager/yumi_manager.h>

using namespace cv;
using namespace std;

YumiManager::YumiManager(std::string point_action_topic, std::string home_action_topic, std::string pick_place_action_topic, std::string camera_topic, std::string controller_type,ros::NodeHandle* nh)
{
    this->point_action_topic = point_action_topic;
    this->home_action_topic = home_action_topic;
    this->pick_place_action_topic = pick_place_action_topic;
    this->camera_topic = camera_topic;

    this->pick_place_client = new PickPlaceClient(this->pick_place_action_topic, true);
    this->point_client = new PointClient(this->point_action_topic, true);
    this->home_client = new HomeClient(this->home_action_topic, true);
    this->image_resized = false;
    this->controller_type = controller_type;

    this->nh = nh;

    this->selected_index = -1;

    this->pick_place_action = true;
    this->point_action = false;
    this->home_action = false;
    this->draw_workspace = false;

    this->plan_action= false;

    //Create a window
    namedWindow("Workspace", 1);

    // std::cout<<camera_topic;

    //set the callback function for any mouse event
    setMouseCallback("Workspace", callbackWorkspace, this);

    createButton("Pick Place",callbackButtonPickPlaceAction,this,CV_RADIOBOX,1);
    createButton("Point",callbackButtonPointAction,this,CV_RADIOBOX,0);
    createButton("Home Position",callbackButtonHomeAction,this,CV_PUSH_BUTTON);
    createButton("Plan Action",callbackButtonPlanAction,this,CV_PUSH_BUTTON);
    createButton("Refresh Scene",callbackButtonRefreshScene,this,CV_PUSH_BUTTON);
    createButton("Show Workspace",callbackButtonShowWorkspace,this,CV_PUSH_BUTTON);



    this->it = new image_transport::ImageTransport(*nh);
    image_transport::Subscriber temp_it_sub = it->subscribe(this->camera_topic, 1, &YumiManager::imageCallback,this);
    this->it_sub =  new image_transport::Subscriber(temp_it_sub);

}
void YumiManager::callbackButtonShowWorkspace(int state, void *userdata)
{
    YumiManager* manager = reinterpret_cast<YumiManager*>(userdata);



    if(!manager->draw_workspace && manager->readWorkspaceConfig("",&manager->workspace_min_x,&manager->workspace_max_x,&manager->workspace_min_y,&manager->workspace_max_y))
    {
        manager->draw_workspace = true;

    }
    else
    {
        manager->draw_workspace = false;
    }


}
bool YumiManager::readWorkspaceConfig(string path, int *minX, int *maxX, int *minY, int *maxY)
{
    string configpath;

    string filename = this->camera_topic;

    std::replace(filename.begin(),filename.end(),'/','_');

    configpath = ROSCppUtils::getHomePath();

    configpath += "/.ros/workspace_segmentation/";

    configpath += "workspace";

    configpath += filename;
    configpath += ".txt";

    ifstream stream(configpath.data());

    if(stream.is_open())
    {
        string str;
        int count = 0;
        while(getline(stream, str))
        {

            std::istringstream ss(str);

            //std::cout<<str<<endl;

            switch(count)
            {
            case 0:
                *minX = atoi(str.data());
            case 1:
                *maxX = atoi(str.data());
            case 2:
                *minY  = atoi(str.data());
            case 3:
                *maxY = atoi(str.data());
            default:
                break;

            }

            count++;

        }


        stream.close();



    }
    else
    {
        return false;
    }

    return true;
}
void YumiManager::drawWorkspace(Mat* img)
{

    Rect rect(workspace_min_x,workspace_min_y,workspace_max_x-workspace_min_x,workspace_max_y-workspace_min_y);

    rectangle(*img,rect,cv::Scalar(0,0,255),2);

}
void YumiManager::callbackWorkspace(int event, int x, int y, int flags, void *userdata)
{
    YumiManager* manager = reinterpret_cast<YumiManager*>(userdata);

    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        int min_sum = 100000;
        int min_index = -1;

        if(manager->image_resized)
        {
            x*=2;
            y*=2;
        }
        for(size_t i =0 ; i < manager->objects.size(); i++)
        {
            int sum = 0;

            sum+= abs(manager->objects[i].pixelposcenterx - x);
            sum+= abs(y - manager->objects[i].pixelposcentery);

            if(sum < min_sum)
            {
                min_sum = sum;
                min_index = i;
            }
        }

        if(min_index >= 0  && min_sum <= 10)
        {
            ROS_INFO("Selected id: %d",min_index);

            manager->selected_index = min_index;


        }


    }

}
void YumiManager::callbackButtonPointAction(int state, void *userdata)
{
    YumiManager* manager = reinterpret_cast<YumiManager*>(userdata);

    if(state == 1)
    {
        manager->pick_place_action = false;
        manager->point_action = true;

        ROS_INFO("Point action is selected");
    }

}
void YumiManager::callbackButtonHomeAction(int state, void *userdata)
{
    YumiManager* manager = reinterpret_cast<YumiManager*>(userdata);


    manager->home_action = true;
    manager->selected_index = -1;

    ROS_INFO("Home position action.");


}

void YumiManager::callbackButtonPlanAction(int state, void *userdata)
{
    YumiManager* manager = reinterpret_cast<YumiManager*>(userdata);

     if (manager->plan_action){

        ROS_WARN("Plan for action disabled!");

        manager->plan_action = false;

    }
    else
     {
        manager->plan_action = true;

        ROS_WARN("Plan for action enabled!");
    }




}
void YumiManager::callbackButtonRefreshScene(int state, void *userdata)
{
    YumiManager* manager = reinterpret_cast<YumiManager*>(userdata);

    manager->objects.clear();

    ros::ServiceClient query_objects_client = manager->nh->serviceClient<perception_manager::QueryObjects>("perception_manager/query_objects");

    perception_manager::QueryObjects query_objects_srv;

    /*if(saveImage())
    {
        ROS_INFO("Image successfully saved!");
    }*/

    if(query_objects_client.call(query_objects_srv))
    {
        manager->objects = query_objects_srv.response.objects ;

        //yumi_manager::SceneObjects so;
        //so.array = objects;
        //scene_publisher.publish(so);

    }
}

void YumiManager::callbackButtonPickPlaceAction(int state, void *userdata)
{
    YumiManager* manager = reinterpret_cast<YumiManager*>(userdata);

    if(state == 1)
    {

        manager->pick_place_action = true;
        manager->point_action = false;
        ROS_INFO("Pick and place action is selected");
    }
}


void YumiManager::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


    Mat temp_img = cv_bridge::toCvShare(msg, "bgr8")->image;

    if(draw_workspace)
    {
        drawWorkspace(&temp_img);
    }
    //ss_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    for(size_t i =0 ; i < this->objects.size(); i++)
    {
        Point pt;
        pt.x = this->objects[i].pixelposcenterx;
        pt.y = this->objects[i].pixelposcentery;

        circle(temp_img, pt, 3, cv::Scalar(0,0,255), -1);
        stringstream ss;
        ss<<this->objects[i].id;
        pt.x +=3;
        pt.x +=3;
        putText(temp_img, ss.str(), pt, 0, 0.4, cv::Scalar(0,0,255), 2);
    }

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
void YumiManager::publishScene(int yumi_status, ros::Publisher* publisher)
{
    yumi_manager::SceneObjects so;

    so.array = this->objects;
    so.yumi_status=yumi_status;
    publisher->publish(so);
}

YumiManager::~YumiManager()
{
    delete pick_place_client;
    delete point_client;
    delete home_client;
    delete it_sub;
    delete it;
}

/*void YumiManager::setHomeActionTopic(std::string topic)
{
    this->home_action_topic = topic;


}*/

