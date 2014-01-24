/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
  * @file sot_filters.cpp
  * @author Gennaro Raiola, Karsten Knese
  * @date December 2013
  * @brief TF Filters for the stack of tasks
  */

#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"

class SotFilters
{

private:
    // Node handle
    ros::NodeHandle nh_;

    // Ros publishers
    ros::Publisher transf_pub;
    ros::Publisher vect_pub;
    geometry_msgs::TransformStamped transf_out;
    geometry_msgs::Vector3Stamped vect_out;

    // Variables necessary to use TF filters, we need filters to define the poses in the target_frame_ (usualy in base_link)
    message_filters::Subscriber<geometry_msgs::TransformStamped> transf_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> vect_sub;
    tf::TransformListener tf_listener_;
    tf::MessageFilter<geometry_msgs::TransformStamped> * transf_filter;
    tf::MessageFilter<geometry_msgs::PoseStamped> * pose_filter;
    tf::MessageFilter<geometry_msgs::Vector3Stamped> * vect_filter;
    std::string target_frame_, param_name_, output_topic_;
    geometry_msgs::PoseStamped pose_in_, pose_out_;

    // Variables used to adapt the transformation
    std::vector<double> quat_param;
    std::vector<double> origin_param;
    tf::Quaternion quat_tmp;
    tf::Vector3 vect_tmp;
    tf::StampedTransform adapt_transf, transf;
    tf::Transform res;

public:
    SotFilters(std::string target_frame, std::string param_name, std::string output_topic):
        nh_("~"),
        tf_listener_(),
        target_frame_(target_frame),
        param_name_(param_name),
        output_topic_(output_topic)
    {

        quat_param.resize(4);
        origin_param.resize(3);

        // Retrieve the transformation from the parameter server
        // This transformation can be used to adapt urdf specified poses into jrl poses.
        ROS_INFO("Loading transformation parameters from %s/quat ",param_name_.c_str());
        nh_.param(param_name_+"/quat/x", quat_param[0], 0.0);
        nh_.param(param_name_+"/quat/y", quat_param[1], 0.0);
        nh_.param(param_name_+"/quat/z", quat_param[2], 0.0);
        nh_.param(param_name_+"/quat/w", quat_param[3], 1.0);
        ROS_INFO("%s/quat/x %f",param_name_.c_str(),quat_param[0]);
        ROS_INFO("%s/quat/y %f",param_name_.c_str(),quat_param[1]);
        ROS_INFO("%s/quat/z %f",param_name_.c_str(),quat_param[2]);
        ROS_INFO("%s/quat/w %f",param_name_.c_str(),quat_param[3]);
        ROS_INFO("Loading transformation parameters from %s/origin ",param_name_.c_str());
        nh_.param(param_name_+"/origin/x", origin_param[0], 0.0);
        nh_.param(param_name_+"/origin/y", origin_param[1], 0.0);
        nh_.param(param_name_+"/origin/z", origin_param[2], 0.0);
        ROS_INFO("%s/origin/x %f",param_name_.c_str(),origin_param[0]);
        ROS_INFO("%s/origin/y %f",param_name_.c_str(),origin_param[1]);
        ROS_INFO("%s/origin/z %f",param_name_.c_str(),origin_param[2]);

        // INPUTS:
        // 1) PoseStamped
        //    - Subscriber
        ROS_INFO("Sot filter subscribed to pose_stamped");
        //nh_.subscribe("pose_stamped",100,poseStampedCallback);
        pose_sub.subscribe(nh_, "pose_stamped", 100);
        //    - TF Filter
        pose_filter = new tf::MessageFilter<geometry_msgs::PoseStamped>(pose_sub, tf_listener_, target_frame_, 100);
        pose_filter->registerCallback(boost::bind(&SotFilters::poseStampedCallback , this, _1));

        // 2) TransformStamped
        //    - Subscriber
        ROS_INFO("Sot filter subscribed to transform_stamped");
        transf_sub.subscribe(nh_, "transform_stamped", 100);
        //    - TF Filter
        transf_filter = new tf::MessageFilter<geometry_msgs::TransformStamped>(transf_sub, tf_listener_, target_frame_, 100);
        transf_filter->registerCallback(boost::bind(&SotFilters::transformStampedCallback, this, _1));

        // 2) Vector3Stamped
        //    - Subscriber
        ROS_INFO("Sot filter subscribed to vector_stamped");
        vect_sub.subscribe(nh_, "vector_stamped", 100);
        //    - TF Filter
        vect_filter = new tf::MessageFilter<geometry_msgs::Vector3Stamped>(vect_sub, tf_listener_, target_frame_, 100);
        vect_filter->registerCallback(boost::bind(&SotFilters::vectorStampedCallback, this, _1));

        // OUTPUTS:
        // 1) TransformStamped
        transf_pub = nh_.advertise<geometry_msgs::TransformStamped>(output_topic_+"_pose", 100);
        // 2) Vector3Stamped
        vect_pub = nh_.advertise<geometry_msgs::Vector3Stamped>(output_topic_+"_position", 100);

    }

    void publish_transf_and_vect(tf::Transform &res, int seq, ros::Time stamp, std::string frame_id){

        transf_out.header.seq = seq;
        transf_out.header.stamp = stamp;
        transf_out.header.frame_id = frame_id.c_str();
        transf_out.transform.translation.x = res.getOrigin().getX();
        transf_out.transform.translation.y = res.getOrigin().getY();
        transf_out.transform.translation.z = res.getOrigin().getZ();
        transf_out.transform.rotation.x = res.getRotation().getX();
        transf_out.transform.rotation.y = res.getRotation().getY();
        transf_out.transform.rotation.z = res.getRotation().getZ();;
        transf_out.transform.rotation.w = res.getRotation().getW();
        transf_pub.publish(transf_out);

        vect_out.header.seq = seq;
        vect_out.header.stamp = stamp;
        vect_out.header.frame_id = frame_id.c_str();
        vect_out.vector.x = res.getOrigin().getX();
        vect_out.vector.y = res.getOrigin().getY();
        vect_out.vector.z = res.getOrigin().getZ();
        vect_pub.publish(vect_out);
    }

    tf::Transform adapt(double x, double y, double z, double qx , double qy, double qz, double qw){
        // Adapt
        quat_tmp.setValue(qx,qy,qz,qw);
        vect_tmp.setValue(x,y,z);
        transf.setRotation(quat_tmp);
        transf.setOrigin(vect_tmp);

        quat_tmp.setValue(quat_param[0],quat_param[1],quat_param[2],quat_param[3]);
        vect_tmp.setValue(origin_param[0],origin_param[1],origin_param[2]);
        adapt_transf.setRotation(quat_tmp);
        adapt_transf.setOrigin(vect_tmp);

        res = transf * adapt_transf;

        return res;
    }

    void TransformStamped2PoseStamped(const geometry_msgs::TransformStamped &in, geometry_msgs::PoseStamped &out){

        out.header = in.header;
        out.pose.position.x = in.transform.translation.x;
        out.pose.position.y = in.transform.translation.y;
        out.pose.position.z = in.transform.translation.z;

        out.pose.orientation.x = in.transform.rotation.x;
        out.pose.orientation.y = in.transform.rotation.y;
        out.pose.orientation.z = in.transform.rotation.z;
        out.pose.orientation.w = in.transform.rotation.w;

    }

    void poseStampedCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& in)
    {

        tf_listener_.transformPose(target_frame_, *in, pose_out_);

        res = adapt(pose_out_.pose.position.x,pose_out_.pose.position.y,pose_out_.pose.position.z,
                    pose_out_.pose.orientation.x,pose_out_.pose.orientation.y,pose_out_.pose.orientation.z,pose_out_.pose.orientation.w);

        publish_transf_and_vect(res,pose_out_.header.seq,pose_out_.header.stamp,pose_out_.header.frame_id);
    }

    void transformStampedCallback(const boost::shared_ptr<const geometry_msgs::TransformStamped>& in)
    {

        TransformStamped2PoseStamped(*in,pose_in_);

        tf_listener_.transformPose(target_frame_, pose_in_, pose_out_);

        res = adapt(pose_out_.pose.position.x,pose_out_.pose.position.y,pose_out_.pose.position.z,
                    pose_out_.pose.orientation.x,pose_out_.pose.orientation.y,pose_out_.pose.orientation.z,pose_out_.pose.orientation.w);

        publish_transf_and_vect(res,pose_out_.header.seq,pose_out_.header.stamp,pose_out_.header.frame_id);

    }

    void vectorStampedCallback(const boost::shared_ptr<const geometry_msgs::Vector3Stamped>& in)
    {
        tf_listener_.transformVector(target_frame_, *in, vect_out);
        vect_pub.publish(vect_out);
    }

};

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "sot_filters");

    // Check and retrieve the arguments
    if (argc!=4){
        ROS_ERROR("Wrong number of arguments");
        ROS_ERROR("Arguments:");
        ROS_ERROR("1) target_frame: frame name for the tf filter");
        ROS_ERROR("2) param_name: name of the parameter containing the transformation to convert poses in jrl convention");
        ROS_ERROR("3) output_topic: name of the output topic");
        ROS_ERROR("   Note: at the end of the output topic will be added a suffix such as _pose or _position");
        return 0;
    }

    // Arguments:
    // 1 target_frame: frame name for the tf filter
    // 2 param_name: name of the parameter containing the transformation to convert poses in jrl convention
    // 3 output_topic: name of the output topic
    //   note: at the end of the name will be added a suffix such as _pose or _position
    SotFilters sot_filter(argv[1],argv[2],argv[3]);

    ros::spin();
}
