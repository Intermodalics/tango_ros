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

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <image_transport/publisher_plugin.h>
#include <nodelet/loader.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

std::string master_uri;
std::string host_ip;

class TangoRosTest : public ::testing::Test {
 public:
  constexpr static int TEST_DURATION = 15; // in second.
  bool connected_to_tango = false;

 protected:
  virtual void SetUp() {
    int argc = 3;
    char* argv[] = {"/", &master_uri[0], &host_ip[0]};
    ros::init(argc, argv, "tango");
    nodelet::Loader loader;
    std::map<std::string, std::string> remappings;
    std::vector<std::string> nodelet_argv;
    LOG(INFO) << "Start loading nodelets.";
    const bool result = loader.load("/tango", "tango_ros_native/TangoRosNode", remappings, nodelet_argv);
    if (!result) {
      LOG(ERROR) << "Problem loading Tango ROS nodelet!";
      return;
    }
    LOG(INFO) << "Finished loading nodelets.";

    // Check that all necessary plugins are available.
    pluginlib::ClassLoader<image_transport::PublisherPlugin> image_transport_pub_loader("image_transport", "image_transport::PublisherPlugin");
    if (!image_transport_pub_loader.isClassAvailable("image_transport/raw_pub")) {
      LOG(ERROR) << "Plugin image_transport/raw_pub is not available.";
      return;
    }
    if (!image_transport_pub_loader.isClassAvailable("image_transport/compressed_pub")) {
      LOG(ERROR) << "Plugin image_transport/compressed_pub is not available.";
      return;
    }
    connected_to_tango = true;

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  }

  virtual void TearDown() {
    if(connected_to_tango) {
      //tango_ros_node_->TangoDisconnect();
    }
  }
};

TEST_F(TangoRosTest, TestPublishingForFixedTime) {
  time_t current_time = time(NULL);
  time_t end = current_time + TEST_DURATION;
  //tango_ros_node_->StartPublishing();
  while(current_time < end) {
    current_time = time(NULL);
  }
  //tango_ros_node_->StopPublishing();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  master_uri = argv[1];
  host_ip = argv[2];
  return RUN_ALL_TESTS();
}
