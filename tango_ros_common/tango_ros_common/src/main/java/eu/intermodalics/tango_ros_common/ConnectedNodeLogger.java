/*
 * Copyright 2016 Intermodalics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package eu.intermodalics.tango_ros_common;

import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;

// When using simulated time, direct or indirect calls to "connectedNode.getCurrentTime()" such
// as connectedNode.getLog().error(e) throw NullPointerExeption.
// See rosjava issue reported here: https://github.com/rosjava/rosjava_core/issues/148
// As a workaround, this class tries to use the rosjava logger and if this throws a NullPointerExeption,
// it uses the android log instead.
public class ConnectedNodeLogger {
    private Log mLog;

    public ConnectedNodeLogger(ConnectedNode connectedNode) {
        mLog = connectedNode.getLog();
    }

    public void debug(String tag, String message) {
        try {
            mLog.debug(message);
        } catch (NullPointerException e) {
            android.util.Log.d(tag, message);
        }
    }

    public void debug(String tag, String message, Throwable t) {
        try {
            mLog.debug(message, t);
        } catch (NullPointerException e) {
            android.util.Log.d(tag, message, t);
        }
    }

    public void info(String tag, String message) {
        try {
            mLog.info(message);
        } catch (NullPointerException e) {
            android.util.Log.i(tag, message);
        }
    }

    public void info(String tag, String message, Throwable t) {
        try {
            mLog.info(message, t);
        } catch (NullPointerException e) {
            android.util.Log.i(tag, message, t);
        }
    }

    public void warn(String tag, String message) {
        try {
            mLog.warn(message);
        } catch (NullPointerException e) {
            android.util.Log.w(tag, message);
        }
    }

    public void warn(String tag, String message, Throwable t) {
        try {
            mLog.warn(message, t);
        } catch (NullPointerException e) {
            android.util.Log.w(tag, message, t);
        }
    }

    public void error(String tag, String message) {
        try {
            mLog.error(message);
        } catch (NullPointerException e) {
            android.util.Log.e(tag, message);
        }
    }

    public void error(String tag, String message, Throwable t) {
        try {
            mLog.error(message, t);
        } catch (NullPointerException e) {
            android.util.Log.e(tag, message, t);
        }
    }

    public void fatal(String tag, String message) {
        try {
            mLog.fatal(message);
        } catch (NullPointerException e) {
            android.util.Log.e(tag, message);
        }
    }

    public void fatal(String tag, String message, Throwable t) {
        try {
            mLog.fatal(message, t);
        } catch (NullPointerException e) {
            android.util.Log.e(tag, message, t);
        }
    }
}
