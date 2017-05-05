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
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

#include <tango_ros_native/tango_ros_node.h>

constexpr double TF_RATE = 150.; // in Hz.
constexpr double POINT_CLOUD_RATE = 4.; // in Hz.
constexpr double FISHEYE_IMAGE_RATE = 25.; // in Hz.
constexpr double COLOR_IMAGE_RATE = 8.; // in Hz.
constexpr double RATE_TOLERANCE_RATIO = 0.2;

constexpr int SLEEP_TIME_UNTIL_FIRST_MESSAGE = 2; // in second.
constexpr double DURATION_RATE_TEST = 10; // in second.

class TangoRosTest : public ::testing::Test {
 public:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber sub_tf_;
  ros::Subscriber sub_point_cloud_;
  image_transport::Subscriber sub_fisheye_image_;
  image_transport::Subscriber sub_color_image_;

  bool tf_message_received_;
  bool point_cloud_received_;
  bool fisheye_image_received_;
  bool color_image_received_;

  int tf_message_count_;
  int point_cloud_message_count_;
  int fisheye_image_message_count_;
  int color_image_message_count_;

  TangoRosTest(): tf_message_received_(false), point_cloud_received_(false),
      fisheye_image_received_(false), color_image_received_(false),
      tf_message_count_(0), point_cloud_message_count_(0),
      fisheye_image_message_count_(0), color_image_message_count_(0) {}

 protected:
  virtual void SetUp() {
    sub_tf_ = nh_.subscribe<tf2_msgs::TFMessage>(
        "/tf", 1, boost::bind(&TangoRosTest::TfCallback, this, _1));

    sub_point_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/tango/" + tango_ros_native::POINT_CLOUD_TOPIC_NAME, 1,
        boost::bind(&TangoRosTest::PointCloudCallback, this, _1));

    image_transport::ImageTransport it(nh_);
    sub_fisheye_image_ =
        it.subscribe("/tango/" + tango_ros_native::FISHEYE_IMAGE_TOPIC_NAME, 1,
                     &TangoRosTest::FisheyeImageCallback, this,
                     image_transport::TransportHints("compressed"));

    sub_color_image_ =
        it.subscribe("/tango/" + tango_ros_native::COLOR_IMAGE_TOPIC_NAME, 1,
                     &TangoRosTest::ColorImageCallback, this,
                     image_transport::TransportHints("compressed"));
  }

 private:
  void TfCallback(const boost::shared_ptr<const tf2_msgs::TFMessage> msg) {
    tf_message_received_ = true;
    tf_message_count_++;
  }

  void PointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2> msg) {
    point_cloud_received_ = true;
    point_cloud_message_count_ ++;
  }

  void FisheyeImageCallback(const sensor_msgs::ImageConstPtr& image) {
    fisheye_image_received_ = true;
    fisheye_image_message_count_++;
  }

  void ColorImageCallback(const sensor_msgs::ImageConstPtr& image) {
    color_image_received_ = true;
    color_image_message_count_++;
  }
};

TEST_F(TangoRosTest, TestMessagesArePublished) {
  // Sleep some time to be sure that data is published.
  sleep(SLEEP_TIME_UNTIL_FIRST_MESSAGE);
  ros::spinOnce();

  tf::StampedTransform transform;
  EXPECT_NO_THROW(tf_listener_.lookupTransform("/start_of_service", "/device",
                                           ros::Time(0), transform));
  EXPECT_NO_THROW(tf_listener_.lookupTransform("/area_description", "/device",
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

TEST_F(TangoRosTest, TestMessageRates) {
  time_t current_time = time(NULL);
  time_t end = current_time + DURATION_RATE_TEST;
  while (current_time < end) {
    ros::spinOnce();
    current_time = time(NULL);
  }
  double tf_rate = tf_message_count_ / DURATION_RATE_TEST;
  double point_cloud_rate = point_cloud_message_count_ / DURATION_RATE_TEST;
  double fisheye_image_rate = fisheye_image_message_count_ / DURATION_RATE_TEST;
  double color_image_rate = color_image_message_count_ / DURATION_RATE_TEST;
  EXPECT_NEAR(TF_RATE, tf_rate, RATE_TOLERANCE_RATIO * TF_RATE);
  EXPECT_NEAR(POINT_CLOUD_RATE, point_cloud_rate, RATE_TOLERANCE_RATIO * POINT_CLOUD_RATE);
  EXPECT_NEAR(FISHEYE_IMAGE_RATE, fisheye_image_rate, RATE_TOLERANCE_RATIO * FISHEYE_IMAGE_RATE);
  EXPECT_NEAR(COLOR_IMAGE_RATE, color_image_rate, RATE_TOLERANCE_RATIO * COLOR_IMAGE_RATE);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tango_ros_test");
  return RUN_ALL_TESTS();
}
