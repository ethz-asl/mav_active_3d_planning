/* Source code for the unrealcv plugin */

#pragma push_macro("check")
#undef check
#include "UnrealCVPrivate.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#pragma pop_macro("check")

namespace ros_node{

    class RosNode {
    public:
        RosNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        virtual ~RosNode() {}

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber sub_;
        ros::Publisher pub_;

        void callback(const std_msgs::String& msg);
    };

    RosNode::RosNode(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private) {
        std::cout << "ROSNODE: Constructor call";
        nh_ = nh;
        nh_private_ = nh_private;
        sub_ = nh_.subscribe("uecvros_plugin_input", 1, &RosNode::callback, this);
        pub_ = nh_private_.advertise<std_msgs::String>("uevros_plugin_output", 1);
    }

    void RosNode::callback(const std_msgs::String &msg) {
        pub_.publish(msg);
    }

    int init(){
        std::cout << "ROSNODE: Init call";
        char *argv[1];
        int argc = 0;
        argv[0] = strdup("");// ./uecvros_plugin_node";
        ros::init(argc, argv, "uecvros_plugin_node");

        ros::NodeHandle nh("");
        ros::NodeHandle nh_private("~");

        ros_node::RosNode uecvros_plugin_node(nh, nh_private);
        ros::spin();
        return 0;
    }

} // namespace ros_node

int main(int argc, char** argv) {
    ros_node::init();
}
