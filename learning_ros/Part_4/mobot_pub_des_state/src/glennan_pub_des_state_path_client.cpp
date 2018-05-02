//ps7 soln:
//starting_pen_pub_des_state_path_client:
// illustrates how to send a request to the append_path_queue_service service
// this version is useful for having mobot exit the starting pen

#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <object_grabber/object_grabberAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_finder/objectFinderAction.h>

#include <object_manipulation_properties/object_ID_codes.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include<generic_gripper_services/genericGripperInterface.h>
using namespace std;

XformUtils xformUtils;
geometry_msgs::PoseStamped g_perceived_object_pose;
int g_found_object_code;
geometry_msgs::PoseStamped object_pickup_poseStamped;
geometry_msgs::PoseStamped object_dropoff_poseStamped;
bool g_got_callback;
actionlib::SimpleActionClient<object_grabber::object_grabberAction> *g_object_grabber_ac_ptr;
int g_object_grabber_return_code;
geometry_msgs::PoseStamped object_poseStamped;

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    g_object_grabber_return_code = result->return_code;
    ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
    g_got_callback=true; //flag that action server has called back
}


void move_to_waiting_pose() {
        ROS_INFO("sending command to move to waiting pose");
        g_got_callback=false; //reset callback-done flag
        object_grabber::object_grabberGoal object_grabber_goal;
        object_grabber_goal.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
        g_object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
}

void grab_object(geometry_msgs::PoseStamped object_pickup_poseStamped) {
    ROS_INFO("sending a grab-object command");
    g_got_callback=false; //reset callback-done flag
    object_grabber::object_grabberGoal object_grabber_goal;
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::GRAB_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = object_pickup_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to grab object: ");
    g_object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
}

void   dropoff_object(geometry_msgs::PoseStamped object_dropoff_poseStamped) {
    ROS_INFO("sending a dropoff-object command");
    object_grabber::object_grabberGoal object_grabber_goal;
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::DROPOFF_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = object_dropoff_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to dropoff object: ");
    g_object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
} 


void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_finder::objectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
        ROS_WARN("object code not recognized");
    }
    else if (g_found_object_code==object_finder::objectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
         g_perceived_object_pose= result->object_pose;
         ROS_INFO("got pose x,y,z = %f, %f, %f",g_perceived_object_pose.pose.position.x,
                 g_perceived_object_pose.pose.position.y,
                 g_perceived_object_pose.pose.position.z);

         ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",g_perceived_object_pose.pose.orientation.x,
                 g_perceived_object_pose.pose.orientation.y,
                 g_perceived_object_pose.pose.orientation.z,
                 g_perceived_object_pose.pose.orientation.w);
         //g_pose_publisher->publish(g_perceived_object_pose);
    }
    else {
        ROS_WARN("object not found!");
    }
}

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

double g_des_state_x=0.0;
double g_des_state_y=0.0;
double g_des_state_phi=0.0;

void desStateCallback(const nav_msgs::Odometry& des_state_rcvd) {
    // copy some of the components of the received message into member vars
    g_des_state_x = des_state_rcvd.pose.pose.position.x;
    g_des_state_y = des_state_rcvd.pose.pose.position.y;
    geometry_msgs::Quaternion quaternion = des_state_rcvd.pose.pose.orientation;
    g_des_state_phi = convertPlanarQuat2Phi(quaternion); // cheap conversion from quaternion to heading for planar motion    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_pub_des_state::path>("append_path_queue_service");
    actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac("object_finder_action_service", true);
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("object_grabber_action_service", true);
    g_object_grabber_ac_ptr = &object_grabber_ac; // make available to fncs

    geometry_msgs::Quaternion quat;
    ros::Subscriber des_state_subscriber= n.subscribe("/desState", 1, desStateCallback);

    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_pub_des_state::path path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 0.0; // say desired x-coord is 5
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(-M_PI/2.0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
 
    pose.position.y = -31.5; //-30.0;
    pose.position.x = -0.2;  
    
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    client.call(path_srv); //
    //monitor progress:
    while (fabs(g_des_state_y-pose.position.y)>0.01) {
        ROS_INFO("des y cmd: %f",g_des_state_y);
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    ROS_INFO("waiting for server: ");
    bool grabber_server_exists = false;
    while ((!grabber_server_exists)&&(ros::ok())) {
        grabber_server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 

    //move to waiting pose
    move_to_waiting_pose();
    while(!g_got_callback) {
        ROS_INFO("waiting on move...");
        ros::Duration(0.5).sleep(); //could do something useful
    }


  object_poseStamped.header.frame_id = "system_ref_frame"; //set object pose; ref frame must be connected via tf
    object_poseStamped.pose.position.x = 0.5;
    object_poseStamped.pose.position.y = -0.35;
    object_poseStamped.pose.position.z = 0.6921; //-0.125; //pose w/rt world frame
    object_poseStamped.pose.orientation.x = 0;
    object_poseStamped.pose.orientation.y = 0;
    object_poseStamped.pose.orientation.z = 0.842;
    object_poseStamped.pose.orientation.w = 0.54;
    object_poseStamped.header.stamp = ros::Time::now();
object_pickup_poseStamped = object_poseStamped;

    grab_object(object_pickup_poseStamped);
    while(!g_got_callback) {
        ROS_INFO("waiting on grab...");
        ros::Duration(0.5).sleep(); //could do something useful
    }    



    ros::Duration(2.0).sleep();
    pose.position.y = 0.0; //-30.0;
    pose.position.x = -0.2;   
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    // let heading settle:
        client.call(path_srv); //
        ros::Duration(2.0).sleep();
        
    path_srv.request.path.poses.clear();

    /*pose.position.y = -30.0;
    pose.position.x = -0.2;   
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);*/
    
    //client.call(path_srv); //
    //monitor progress:
    while (fabs(g_des_state_y+30.0)>0.01) {
        ROS_INFO("des y cmd: %f",g_des_state_y);
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    
    ROS_WARN("do manipulation here...");
    ROS_INFO("waiting for server: ");
    
    /*bool finder_server_exists = false;
    while ((!finder_server_exists)&&(ros::ok())) {
        finder_server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    //ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    //g_pose_publisher = &pose_publisher;
    object_finder::objectFinderGoal object_finder_goal;
    //object_finder::objectFinderResult object_finder_result;

    object_finder_goal.object_id = ObjectIdCodes::TABLE_SURFACE;
    object_finder_goal.known_surface_ht = false; //require find table height
    //object_finder_goal.object_id=object_finder::objectFinderGoal::COKE_CAN_UPRIGHT;
    //object_finder_goal.object_id=object_finder::objectFinderGoal::TOY_BLOCK;
    //object_finder_goal.known_surface_ht=true;
    //object_finder_goal.known_surface_ht=false; //require find table height
    //object_finder_goal.surface_ht = 0.05;
    double surface_height;
    ROS_INFO("sending goal: ");
        object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
        
        bool finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
        }
        
    if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND) {
                        ROS_INFO("surface-finder success");
                        surface_height = g_perceived_object_pose.pose.position.z; // table-top height, as found by object_finder 
                        ROS_INFO("found table ht = %f",surface_height);   }
    else {
        ROS_WARN("did not find table height; quitting:");
    }
    //if here, then find block using known table height:
    object_finder_goal.known_surface_ht = true;
    object_finder_goal.surface_ht = surface_height;
    ROS_INFO("using surface ht = %f",surface_height);        
    object_finder_goal.object_id=ObjectIdCodes::TOY_BLOCK_ID;
     ROS_INFO("sending goal to find TOY_BLOCK: ");
        object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
        
        finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
        }       
        
    if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND)   {
        ROS_INFO("found object!");
    }    
    else {
        ROS_WARN("object not found!:");
    }*/

    
  /*  ROS_INFO("head home: ");
    
    path_srv.request.path.poses.clear();
    pose.position.y = -28.8;
    pose.position.x = -0.2;   
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);    
        client.call(path_srv); //

    //let settle:
        ROS_WARN("turn and let settle");
        ros::Duration(2.0).sleep();
    pose.position.y = 0;
    pose.position.x = -0.2;   
    pose_stamped.pose = pose;
    path_srv.request.path.poses.clear();

    path_srv.request.path.poses.push_back(pose_stamped);      

    client.call(path_srv); //
    ROS_WARN("head home!");
    while (fabs(g_des_state_y)>0.01) {
        ROS_INFO("des y cmd: %f",g_des_state_y);
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }*/
    ROS_INFO("done!!");
    return 0;
}
