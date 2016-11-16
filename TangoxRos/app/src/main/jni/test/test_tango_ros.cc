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
#include <tango_ros_native/tango_ros_node.h>
#include <tango_ros_native/tango_ros_util.h>

#include <gtest/gtest.h>

class TangoRosTest : public ::testing::Test {
 public:
  const std::string MASTER_URI = "__master:=http://im-desktop-005:11311";
  const std::string DEVICE_IP = "__ip:=192.168.168.184";
  std::shared_ptr<tango_ros_node::TangoRosNode> tango_ros_node_;

 protected:
  virtual void SetUp() {
    tango_ros_util::InitRos(MASTER_URI.c_str(), DEVICE_IP.c_str());
    publisher_config_.publish_device_pose = true;
    publisher_config_.publish_point_cloud = true;
    publisher_config_.publish_camera = tango_ros_node::CAMERA_FISHEYE | tango_ros_node::CAMERA_COLOR;
    tango_ros_node_.reset(new tango_ros_node::TangoRosNode(publisher_config_));
    tango_ros_node_->OnTangoServiceConnected();
  }

  virtual void TearDown() {
    tango_ros_node_->TangoDisconnect();
  }

 private:
  tango_ros_node::PublisherConfiguration publisher_config_;
};

TEST_F(TangoRosTest, Test1) {
  while(tango_ros_util::IsRosOK()) {
    tango_ros_node_->Publish();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}