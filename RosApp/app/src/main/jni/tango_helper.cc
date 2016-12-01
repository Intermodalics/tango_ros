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
#include "tango_helper.h"

#include <glog/logging.h>
#include <tango_client_api/tango_client_api.h>
#include <tango_support_api/tango_support_api.h>

namespace tango_helper {
bool IsTangoVersionOk(JNIEnv* env, jobject activity) {
  int version;
  TangoErrorType err = TangoSupport_GetTangoVersion(env, activity, &version);
  if (err != TANGO_SUCCESS || version < TANGO_CORE_MINIMUM_VERSION) {
    LOG(ERROR) << "Tango Core version is out of"
        "date, minimum version required: " << TANGO_CORE_MINIMUM_VERSION <<
        ", version used: " << version;
    return false;
  }
  return true;
}

bool SetBinder(JNIEnv* env, jobject binder) {
  TangoErrorType ret = TangoService_setBinder(env, binder);
  if (ret != TANGO_SUCCESS) {
    LOG(ERROR) << "Failed to bind Tango service with error code: " << ret;
    return false;
  }
  return true;
}
} // tango_helper
