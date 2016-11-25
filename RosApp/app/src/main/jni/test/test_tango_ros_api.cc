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
#include <tango_ros_native/tango_ros_node.h>
#include <tango_ros_native/tango_ros_util.h>

std::string master_uri;
std::string device_ip;

class TangoRosTest : public ::testing::Test {
 public:
  const int TEST_DURATION = 5; // in second.
  std::shared_ptr<tango_ros_node::TangoRosNode> tango_ros_node_;
  tango_ros_node::PublisherConfiguration publisher_config_;
  bool connected_to_tango = false;

 protected:
  virtual void SetUp() {
    ASSERT_TRUE(tango_ros_util::InitRos(master_uri.c_str(), device_ip.c_str()));
    publisher_config_.publish_device_pose = true;
    publisher_config_.publish_point_cloud = true;
    publisher_config_.publish_camera = tango_ros_node::CAMERA_FISHEYE | tango_ros_node::CAMERA_COLOR;
    tango_ros_node_.reset(new tango_ros_node::TangoRosNode(publisher_config_));
    ASSERT_TRUE(tango_ros_node_->OnTangoServiceConnected());
    connected_to_tango = true;
  }

  virtual void TearDown() {
    if(connected_to_tango) {
      tango_ros_node_->TangoDisconnect();
    }
  }
};

TEST_F(TangoRosTest, TestPublishingForFixedTime) {
  time_t current_time = time(NULL);
  time_t end = current_time + TEST_DURATION;
  tango_ros_node_->StartPublishingThreads();
  while(current_time < end) {
    current_time = time(NULL);
  }
  tango_ros_node_->StopPublishingThreads();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  master_uri = argv[1];
  device_ip = argv[2];
  return RUN_ALL_TESTS();
}
