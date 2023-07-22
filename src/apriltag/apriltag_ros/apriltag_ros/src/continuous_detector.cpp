/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));
  listener=std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(this->buffer));

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");

  int queue_size;
  pnh.param<int>("queue_size", queue_size, 1);
  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", queue_size,
                          &ContinuousDetector::imageCallback, this,
                          image_transport::TransportHints(transport_hint));
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);

  referee_tag_pub_ = nh.advertise<referee_msgs::Apriltag_info>("/apriltag_detection",10);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  refresh_params_service_ =
      pnh.advertiseService("refresh_tag_params", 
                          &ContinuousDetector::refreshParamsCallback, this);
  for(int i=0;i<8;i++)
    for(int j=0;j<4;j++)
      pt[i][j]=0;
}

void ContinuousDetector::refreshTagParameters()
{
  // Resetting the tag detector will cause a new param server lookup
  // So if the parameters have changed (by someone/something), 
  // they will be updated dynamically
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  ros::NodeHandle& pnh = getPrivateNodeHandle();
  tag_detector_.reset(new TagDetector(pnh));
}

bool ContinuousDetector::refreshParamsCallback(std_srvs::Empty::Request& req,
                                               std_srvs::Empty::Response& res)
{
  refreshTagParameters();
  return true;
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  auto tags= tag_detector_->detectTags(cv_image_,camera_info);
  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_.publish(
      tags);
  if(!tags.detections.empty()){
    auto tag_detection=tags.detections.at(0);
    int id=tag_detection.id.at(0);
    geometry_msgs::PoseWithCovarianceStamped temp_pose=tag_detection.pose;
    geometry_msgs::PoseStamped pose_stamp_camera,pose_stamp_world;
    pose_stamp_camera.header=temp_pose.header;
    pose_stamp_camera.pose=temp_pose.pose.pose;
    pose_stamp_world=buffer.transform(pose_stamp_camera,"world");
    if(pt[id][3]==0){
          num+=1;
    }
    std::cout<<"pose stamp "<<" num  "<<num<<"  "<<pose_stamp_world<<std::endl;
    pt[id][0]+=pose_stamp_world.pose.position.x;
    pt[id][1]+=pose_stamp_world.pose.position.y;
    pt[id][2]+=pose_stamp_world.pose.position.z;
    pt[id][3]+=1.0;

    std::cout<< " x  "<<pt[id][0]/pt[id][3]<<"  "<<
                " y  "<<pt[id][1]/pt[id][3]<<"  "<<
                " z  "<<pt[id][2]/pt[id][3]<<"  "<<std::endl;
    if((last_id!=id ||pt[last_id][3]>=10||(num==8&&pt[last_id][3]>=3))&&pt[last_id][3]>0){
        referee_msgs::Apriltag_info msg;
        msg.tag_num = last_id;
        msg.tag_pos_x = pt[last_id][0]/pt[last_id][3];
        msg.tag_pos_y = pt[last_id][1]/pt[last_id][3];
        msg.tag_pos_z = pt[last_id][2]/pt[last_id][3];
        pt[last_id][3]=-500;
        referee_tag_pub_.publish(msg);
    }
    last_id=id;
    // if(flag[id]!=-1){
    //   flag[id]+=1;
    //   if(last_id==id){
    //       pt[0] += pose_stamp_world.pose.position.x;
    //       pt[1] += pose_stamp_world.pose.position.y;
    //       pt[2] += pose_stamp_world.pose.position.z;
    //   }
    //   return;
    // }
    // flag[id]=1;
    // geometry_msgs::PoseWithCovarianceStamped temp_pose=tag_detection.pose;
    // geometry_msgs::PoseStamped pose_stamp_camera,pose_stamp_world;
    // pose_stamp_camera.header=temp_pose.header;
    // pose_stamp_camera.pose=temp_pose.pose.pose;
    // pose_stamp_world=buffer.transform(pose_stamp_camera,"world");
    // std::cout<<"pose stamp "<<" num  "<<num<<"  "<<pose_stamp_world<<std::endl;
    // referee_msgs::Apriltag_info msg;
    // msg.tag_num = id;
    // msg.tag_pos_x = pose_stamp_world.pose.position.x;
    // msg.tag_pos_y = pose_stamp_world.pose.position.y;
    // msg.tag_pos_z = pose_stamp_world.pose.position.z;
    // referee_tag_pub_.publish(msg);

  }

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros
