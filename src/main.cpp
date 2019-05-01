#include <ros/ros.h>
#include <iostream>

#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <acl_msgs/State.h>
#include <tag_state_sync/TagState.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class Node
{

public:

    Node()
    {
        std::string name;
        ros::param::param<std::string>("~name",name,"SQ01");
        pub_pair = node.advertise<tag_state_sync::TagState>("/" + name + "/tag_state", 1);

        subs_tag.subscribe(node, "/tag_detections", 1);
        subs_state.subscribe(node, "/" + name + "/state",1);

        sync.reset(new Sync(MySyncPolicy(1000), subs_tag, subs_state));
        sync->registerCallback(boost::bind(&Node::callback, this, _1, _2));

        std::cout << "Starting tag_state_sync for drone " << name << std::endl;
    }

    void callback(const apriltags2_ros::AprilTagDetectionArrayConstPtr& detection, const acl_msgs::StateConstPtr state)
    {
        //std::cout << "entered inside " << std::endl;
        //std::cout << "Msg: " << _msg->DebugString() << std::endl;

        tag_state_sync::TagState tagState;
        tagState.header.frame_id = "world";
        //tagState.header.stamp = ros::Time::now();  // TODO: test if it needs to be created explicitly
        tagState.tag_detection = *detection;
        tagState.state = *state;

        pub_pair.publish(tagState);
    }

private:

    ros::NodeHandle node;
    ros::Publisher pub_pair;

    message_filters::Subscriber<apriltags2_ros::AprilTagDetectionArray> subs_tag;
    message_filters::Subscriber<acl_msgs::State> subs_state;

    typedef message_filters::sync_policies::ApproximateTime<apriltags2_ros::AprilTagDetectionArray, acl_msgs::State> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_sync");
    Node synchronizer;

    ros::spin();
}
