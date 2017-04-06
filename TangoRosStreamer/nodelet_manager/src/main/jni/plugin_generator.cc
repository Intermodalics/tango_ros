// Copyright 2017 Intermodalics All Rights Reserved.
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
#include <map>
#include <string>

#include <class_loader/multi_library_class_loader.h>
#include <pluginlib/class_desc.h>

using namespace pluginlib;

// Lists all plugins that can be loaded by the nodelet manager in this app.
typedef std::map<std::string, ClassDesc> classes_available_map;
typedef std::pair<std::string, ClassDesc> plugin_pair;
classes_available_map getStaticClassesAvailable(void) {
  classes_available_map pluginClasses;
  pluginClasses.insert(
        plugin_pair(
          "image_transport/compressedDepth_pub",
          ClassDesc("image_transport/compressedDepth_pub",
                    "compressed_depth_image_transport::CompressedDepthPublisher",
                    "image_transport::PublisherPlugin",
                    "compressed_depth_image_transport",
                    "This plugin publishes a compressed depth images using PNG compression.",
                    "lib/libcompressed_depth_image_transport",
                    "")));
  pluginClasses.insert(
        plugin_pair(
          "image_transport/compressedDepth_sub",
          ClassDesc("image_transport/compressedDepth_sub",
                    "compressed_depth_image_transport::CompressedDepthSubscriber",
                    "image_transport::SubscriberPlugin",
                    "compressed_depth_image_transport",
                    "This plugin decodes a compressed depth images.",
                    "lib/libcompressed_depth_image_transport",
                    "")));
  pluginClasses.insert(
        plugin_pair(
          "image_transport/compressed_pub",
          ClassDesc("image_transport/compressed_pub",
                    "compressed_image_transport::CompressedPublisher",
                    "image_transport::PublisherPlugin",
                    "compressed_image_transport",
                    "This plugin publishes a CompressedImage using either JPEG or PNG compression.",
                    "lib/libcompressed_image_transport",
                    "")));
  pluginClasses.insert(
        plugin_pair(
          "image_transport/compressed_sub",
          ClassDesc("image_transport/compressed_sub",
                    "compressed_image_transport::CompressedSubscriber",
                    "image_transport::SubscriberPlugin",
                    "compressed_image_transport",
                    "This plugin decompresses a CompressedImage topic.",
                    "lib/libcompressed_image_transport",
                    "")));
  pluginClasses.insert(
        plugin_pair(
          "tango_ros_native/TangoRosNode",
          ClassDesc("tango_ros_native/TangoRosNode",
                    "tango_ros_native::TangoRosNode",
                    "nodelet::Nodelet",
                    "tango_ros_native",
                    "lib/libtango_ros_native",
                    "Publishes Tango sensor data.",
                    "")));
  return pluginClasses;
}
