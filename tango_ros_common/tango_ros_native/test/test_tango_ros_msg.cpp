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
#include <time.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

#include <tango_ros_native/tango_ros_node.h>
#include <tango_ros_native/tango_ros_util.h>

const double TF_RATE = 100; // in Hz.
const double POINT_CLOUD_RATE = 5.; // in Hz.
const double FISHEYE_IMAGE_RATE = 20.; // in Hz.
const double COLOR_IMAGE_RATE = 10.; // in Hz.
const double RATE_TOLERANCE_PERCENTAGE = 0.3;

class TangoRosTest : public ::testing::Test {
 public:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber sub_tf_;
  ros::Subscriber sub_point_cloud_;
  ros::Subscriber sub_fisheye_image_;
  ros::Subscriber sub_color_image_;

  bool tf_message_received_;
  bool point_cloud_received_;
  bool fisheye_image_received_;
  bool color_image_received_;

  int tf_message_count;
  int point_cloud_message_count;
  int fisheye_image_message_count;
  int color_image_message_count;

 protected:
  virtual void SetUp() {
    tf_message_received_ = false;
    point_cloud_received_ = false;
    fisheye_image_received_ = false;
    color_image_received_ = false;
    tf_message_count = 0;
    point_cloud_message_count = 0;
    fisheye_image_message_count = 0;
    color_image_message_count = 0;

    sub_point_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        tango_ros_native::PublisherConfiguration().point_cloud_topic, 1,
        boost::bind(&TangoRosTest::PointCloudCallback, this, _1));

    sub_fisheye_image_ = nh_.subscribe<sensor_msgs::CompressedImage>(
        tango_ros_native::PublisherConfiguration().fisheye_camera_topic, 1,
            boost::bind(&TangoRosTest::FisheyeImageCallback, this, _1));

    sub_color_image_ = nh_.subscribe<sensor_msgs::CompressedImage>(
        tango_ros_native::PublisherConfiguration().color_camera_topic, 1,
            boost::bind(&TangoRosTest::ColorImageCallback, this, _1));

    sub_tf_ = nh_.subscribe<tf2_msgs::TFMessage>(
        "/tf", 1, boost::bind(&TangoRosTest::TfCallback, this, _1));
  }

 private:
  void TfCallback(const boost::shared_ptr<const tf2_msgs::TFMessage> msg) {
    tf_message_received_ = true;
    tf_message_count++;
  }

  void PointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2> msg) {
    point_cloud_received_ = true;
    point_cloud_message_count ++;
  }

  void FisheyeImageCallback(const boost::shared_ptr<const sensor_msgs::CompressedImage> msg) {
    fisheye_image_received_ = true;
    fisheye_image_message_count++;
  }

  void ColorImageCallback(const boost::shared_ptr<const sensor_msgs::CompressedImage> msg) {
    color_image_received_ = true;
    color_image_message_count++;
  }
};

TEST_F(TangoRosTest, TestMessagesArePublished) {
  tf::StampedTransform transform;
  EXPECT_THROW(tf_listener_.lookupTransform("/start_of_service", "/device",
                                            ros::Time(0), transform), tf::TransformException);
  EXPECT_THROW(tf_listener_.lookupTransform("/device", "/camera_depth",
                                              ros::Time(0), transform), tf::TransformException);
  EXPECT_THROW(tf_listener_.lookupTransform("/device", "/camera_fisheye",
                                              ros::Time(0), transform), tf::TransformException);
  EXPECT_THROW(tf_listener_.lookupTransform("/device", "/camera_color",
                                              ros::Time(0), transform), tf::TransformException);
  EXPECT_FALSE(tf_message_received_);
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
  EXPECT_TRUE(tf_message_received_);
  EXPECT_TRUE(point_cloud_received_);
  EXPECT_TRUE(fisheye_image_received_);
  EXPECT_TRUE(color_image_received_);
}

TEST_F(TangoRosTest, TestMessagesRates) {
  double duration = 5;
  time_t current_time = time(NULL);
  time_t end = current_time + duration;
  while (current_time < end) {
    ros::spinOnce();
    current_time = time(NULL);
  }
  double tf_rate = tf_message_count / duration;
  double point_cloud_rate = point_cloud_message_count / duration;
  double fisheye_image_rate = fisheye_image_message_count / duration;
  double color_image_rate = color_image_message_count / duration;
  EXPECT_NEAR(TF_RATE, tf_rate, RATE_TOLERANCE_PERCENTAGE * TF_RATE);
  EXPECT_NEAR(POINT_CLOUD_RATE, point_cloud_rate, RATE_TOLERANCE_PERCENTAGE * POINT_CLOUD_RATE);
  EXPECT_NEAR(FISHEYE_IMAGE_RATE, fisheye_image_rate, RATE_TOLERANCE_PERCENTAGE * FISHEYE_IMAGE_RATE);
  EXPECT_NEAR(COLOR_IMAGE_RATE, color_image_rate, RATE_TOLERANCE_PERCENTAGE * COLOR_IMAGE_RATE);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tango_ros_test");
  return RUN_ALL_TESTS();
}
