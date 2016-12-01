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
#include <jni.h>

// Helper functions that need to be separated from the tango_ros_native package to get rif of the
// jni dependency, so that the tango_ros_native package can be used for Android and Linux.
namespace tango_helper {
// The minimum Tango Core version required from this application.
const int TANGO_CORE_MINIMUM_VERSION = 11926; // Yildun release.

// Checks the installed version of the TangoCore. If it is too old, then
// it will not support the most up to date features.
// @return returns true if tango version if supported.
bool IsTangoVersionOk(JNIEnv* env, jobject activity);
// Binds to the tango service.
// @return returns true if setting the binder ended successfully.
bool SetBinder(JNIEnv* env, jobject binder);
} // tango_helper

