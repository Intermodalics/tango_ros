// Copyright 2016 Intermodalics All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <tango_ros_native/tango_ros_node.h>
#include <tango_ros_native/tango_ros_util.h>

class TangoRosTest : public ::testing::Test {
 public:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber sub_point_cloud_;
  ros::Subscriber sub_fisheye_image_;
  ros::Subscriber sub_color_image_;

  bool point_cloud_received_ = false;
  bool fisheye_image_received_ = false;
  bool color_image_received_ = false;

 protected:
  virtual void SetUp() {
    sub_point_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        tango_ros_node::PublisherConfiguration().point_cloud_topic, 1,
        boost::bind(&TangoRosTest::PointCloudCallback, this, _1));

    sub_fisheye_image_ = nh_.subscribe<sensor_msgs::CompressedImage>(
            tango_ros_node::PublisherConfiguration().fisheye_camera_topic, 1,
            boost::bind(&TangoRosTest::FisheyeImageCallback, this, _1));

    sub_color_image_ = nh_.subscribe<sensor_msgs::CompressedImage>(
            tango_ros_node::PublisherConfiguration().color_camera_topic, 1,
            boost::bind(&TangoRosTest::ColorImageCallback, this, _1));
  }

 private:
  void PointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2> msg) {
    point_cloud_received_ = true;
  }

  void FisheyeImageCallback(const boost::shared_ptr<const sensor_msgs::CompressedImage> msg) {
    fisheye_image_received_ = true;
  }

  void ColorImageCallback(const boost::shared_ptr<const sensor_msgs::CompressedImage> msg) {
    color_image_received_ = true;
  }
};

TEST_F(TangoRosTest, TestMessages) {
  tf::StampedTransform transform;
  EXPECT_THROW(tf_listener_.lookupTransform("/start_of_service", "/device",
                                            ros::Time(0), transform), tf::TransformException);
  EXPECT_THROW(tf_listener_.lookupTransform("/device", "/camera_depth",
                                              ros::Time(0), transform), tf::TransformException);
  EXPECT_THROW(tf_listener_.lookupTransform("/device", "/camera_fisheye",
                                              ros::Time(0), transform), tf::TransformException);
  EXPECT_THROW(tf_listener_.lookupTransform("/device", "/camera_color",
                                              ros::Time(0), transform), tf::TransformException);
  EXPECT_FALSE(point_cloud_received_);
  EXPECT_FALSE(fisheye_image_received_);
  EXPECT_FALSE(color_image_received_);

  // Sleep some time to be sure that data is published.
  sleep(2);
  ros::spinOnce();

  EXPECT_NO_THROW(tf_listener_.lookupTransform("/start_of_service", "/device",
                                           ros::Time(0), transform));
  EXPECT_NO_THROW(tf_listener_.lookupTransform("/device", "/camera_depth",
                                             ros::Time(0), transform));
  EXPECT_NO_THROW(tf_listener_.lookupTransform("/device", "/camera_fisheye",
                                             ros::Time(0), transform));
  EXPECT_NO_THROW(tf_listener_.lookupTransform("/device", "/camera_color",
                                             ros::Time(0), transform));
  EXPECT_TRUE(point_cloud_received_);
  EXPECT_TRUE(fisheye_image_received_);
  EXPECT_TRUE(color_image_received_);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tango_ros_test");
  return RUN_ALL_TESTS();
}
