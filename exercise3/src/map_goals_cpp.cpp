#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0;
tf::Transform map_transform;
int size_y = 0;

ros::Publisher goal_pub;
ros::Subscriber map_sub;
ros::Publisher goal_request_pub;
ros::Subscriber goal_response_sub;

void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    int size_x = msg_map->info.width;
    size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3))
    {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ((cv_map.rows != size_y) && (cv_map.cols != size_x))
    {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
    tf::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t> &map_msg_data(msg_map->data);

    unsigned char *cv_map_data = (unsigned char *)cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y - 1;

    for (int y = size_y_rev; y >= 0; --y)
    {

        int idx_map_y = size_x * (size_y - y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x)
        {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }
}

void mouseCallback(int event, int x, int y, int, void *data)
{

    if (event != EVENT_LBUTTONDOWN || cv_map.empty())
        return;

    int v = (int)cv_map.at<unsigned char>(y, x);

    if (v != 255)
    {
        ROS_WARN("Unable to move to (x: %d, y: %d), not reachable", x, y);
        return;
    }

    ROS_INFO("Moving to (x: %d, y: %d)", x, y);

    tf::Point pt((float)x * map_resolution, (float)y * map_resolution, 0.0);
    tf::Point transformed = map_transform * pt;

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1;
    goal.pose.position.x = transformed.x();
    goal.pose.position.y = -transformed.y();
    goal.header.stamp = ros::Time::now();

    goal_pub.publish(goal);
}

void goalCallback(const geometry_msgs::Point &point)
{

    int v = (int)cv_map.at<unsigned char>(point.y, point.x);


    if (v != 255)
    {
        return;
    }

    tf::Point pt((float)point.x * map_resolution, (float)(size_y - point.y) * map_resolution, 0.0);
    tf::Point transformed = map_transform * pt;

    ROS_INFO("gC -> Moving to (x: %d, y: %d)", (int)point.x, (int)point.y);

    ROS_INFO("gC -> Moving to (x: %f, y: %f)", transformed.x(), transformed.y());


    /*** Sending MoveBaseGoal with ActionClient to /move_base/goal ***/
    // TODO: reuse an instance of MoveBaseClient
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1;
    goal.pose.position.x = transformed.x();
    goal.pose.position.y = -transformed.y();
    goal.header.stamp = ros::Time::now();

    move_base_msgs::MoveBaseGoal goal1;
    goal1.target_pose = goal;

    ac.sendGoal(goal1);
    ac.waitForResult(ros::Duration(60));
    ROS_INFO("Action result: %s", ac.getState().toString().c_str());

    /*** Sending Point to /move_base_simple/goal ***/
    // goal_pub.publish(goal);

    ROS_INFO("Request a goal. Current point: %d,%d\n", (int)point.x, (int)point.y);
    goal_request_pub.publish(point);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_goals_cpp");
    ros::NodeHandle n;

    map_sub = n.subscribe("map", 10, &mapCallback);
    goal_response_sub = n.subscribe("goal/response", 10, &goalCallback);

    goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);
    goal_request_pub = n.advertise<geometry_msgs::Point>("goal/request", 10);

    ros::Rate loop_rate(1);
    loop_rate.sleep();

    geometry_msgs::Point p;
    p.x = 5.0;
    p.y = 2.0;
    p.z = 0.0;
    ROS_INFO("Publishing to (x: %d, y: %d)", (int)p.x, (int)p.y);
    goal_request_pub.publish(p);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}