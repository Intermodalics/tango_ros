/*
 * Copyright 2016 Google Inc. All Rights Reserved.
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

package com.rosjava.tangoxros;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.util.Log;

import java.io.File;

/**
 * Functions for simplifying the process of initializing TangoService, and function
 * handles loading correct libtango_client_api.so.
 */
public class TangoInitializationHelper {
    public static final int ARCH_ERROR = -2;
    public static final int ARCH_FALLBACK = -1;
    public static final int ARCH_DEFAULT = 0;
    public static final int ARCH_ARM64 = 1;
    public static final int ARCH_ARM32 = 2;
    public static final int ARCH_X86_64 = 3;
    public static final int ARCH_X86 = 4;

    protected static boolean mIsTangoServiceBound;
    protected static boolean mIsTangoVersionOk;

    static {
        try {
            System.loadLibrary(TangoRosNode.DEFAULT_LIB_NAME);
        } catch (Exception e) {
            Log.e(TangoInitializationHelper.class.getName(),
                    "Error loading library " + TangoRosNode.DEFAULT_LIB_NAME, e);
        }
    }

    /**
     * Binds to the tango service.
     *
     * @param nativeTangoServiceBinder The native binder object.
     * @return true if binding to the Tango service ended successfully.
     */
    protected static native boolean setBinderTangoService(IBinder nativeTangoServiceBinder);

    /**
     * Check if the tango version is correct.
     *
     * @param callerActivity the caller activity of this function.
     * @return true if the version of tango is ok.
     */
    protected static native boolean isTangoVersionOk(Activity callerActivity);

    // Getters for Tango Status variables
    public static boolean isTangoServiceBound() {
        return mIsTangoServiceBound;
    }

    public static boolean isTangoVersionOk() {
        return mIsTangoVersionOk;
    }

    /**
     * Only for apps using the C API:
     * Initializes the underlying TangoService for native apps.
     *
     * @return returns false if the device doesn't have the Tango running as Android Service.
     *     Otherwise ture.
     */
    public static final boolean bindTangoService(final Context context,
                                                 ServiceConnection connection) {
        Intent intent = new Intent();
        intent.setClassName("com.google.tango", "com.google.atap.tango.TangoService");

        boolean hasJavaService = (context.getPackageManager().resolveService(intent, 0) != null);

        // User doesn't have the latest packagename for TangoCore, fallback to the previous name.
        if (!hasJavaService) {
            intent = new Intent();
            intent.setClassName("com.projecttango.tango", "com.google.atap.tango.TangoService");
            hasJavaService = (context.getPackageManager().resolveService(intent, 0) != null);
        }

        // User doesn't have a Java-fied TangoCore at all; fallback to the deprecated approach
        // of doing nothing and letting the native side auto-init to the system-service version
        // of Tango.
        if (!hasJavaService) {
            return false;
        }

        return context.bindService(intent, connection, Context.BIND_AUTO_CREATE);
    }


    /**
     * Load the libtango_client_api.so library based on different Tango device setup.
     *
     * @return returns the loaded architecture id.
     */
    public static final int loadTangoSharedLibrary() {
        int loadedSoId = ARCH_ERROR;
        String basePath = "/data/data/com.google.tango/libfiles/";
        if (!(new File(basePath).exists())) {
            basePath = "/data/data/com.projecttango.tango/libfiles/";
        }
        Log.i("TangoInitializationHelper", "basePath: " + basePath);

        try {
            System.load(basePath + "arm64-v8a/libtango_client_api.so");
            loadedSoId = ARCH_ARM64;
            Log.i("TangoInitializationHelper", "Success! Using arm64-v8a/libtango_client_api.");
        } catch (UnsatisfiedLinkError e) {
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.load(basePath + "armeabi-v7a/libtango_client_api.so");
                loadedSoId = ARCH_ARM32;
                Log.i("TangoInitializationHelper", "Success! Using armeabi-v7a/libtango_client_api.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.load(basePath + "x86_64/libtango_client_api.so");
                loadedSoId = ARCH_X86_64;
                Log.i("TangoInitializationHelper", "Success! Using x86_64/libtango_client_api.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.load(basePath + "x86/libtango_client_api.so");
                loadedSoId = ARCH_X86;
                Log.i("TangoInitializationHelper", "Success! Using x86/libtango_client_api.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.load(basePath + "default/libtango_client_api.so");
                loadedSoId = ARCH_DEFAULT;
                Log.i("TangoInitializationHelper", "Success! Using default/libtango_client_api.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.loadLibrary("tango_client_api");
                loadedSoId = ARCH_FALLBACK;
                Log.i("TangoInitializationHelper", "Falling back to libtango_client_api.so symlink.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        return loadedSoId;
    }

    /**
     * Creates a DefaultTangoServiceConnection with no callback.
     * @return New ServiceConnection handler that can be used with {@link #bindTangoService}
     */
    public static ServiceConnection NewDefaultServiceConnection() {
        return new DefaultTangoServiceConnection(null);
    }

    /**
     * Creates a DefaultTangoServiceConnection with a custom callback.
     * @param callback Function to be called after connection attempt is finished.
     * @return New ServiceConnection handler that can be used with {@link #bindTangoService}
     */
    public static ServiceConnection NewDefaultServiceConnection(
            DefaultTangoServiceConnection.AfterConnectionCallback callback) {
        return new DefaultTangoServiceConnection(callback);
    }

    /**
     * Checks if Tango Version is ok using native functions, and sets status variable.
     * @param activity Caller activity.
     * @return True if Tango Version is Ok.
     */
    public static boolean checkTangoVersionOk(Activity activity) {
        mIsTangoVersionOk = isTangoVersionOk(activity);
        return mIsTangoVersionOk;
    }

    /**
     * Class that simplifies the second stage of the Tango binding process.
     * An instance of this connection can be sent to {@link #bindTangoService} for default behaviour.
     * A callback can be executed by overriding {@link AfterConnectionCallback} when creating
     * an instance of this object. At that point, state variables {@link #mIsTangoServiceBound} and
     * {@link #mIsTangoVersionOk} should be already set, so that different cases can be handled.
     */
    public static class DefaultTangoServiceConnection implements ServiceConnection {

        private AfterConnectionCallback connectionErrorCallback;

        public DefaultTangoServiceConnection(AfterConnectionCallback callback) {
            connectionErrorCallback = callback;
        }

        public static interface AfterConnectionCallback {
            public void execute();
        }

        public void onServiceConnected(ComponentName name, IBinder service) {
            // Synchronization around RunningActivity object is to avoid
            // Tango disconnect in the middle of the connecting operation.
            mIsTangoServiceBound = setBinderTangoService(service);

            if (connectionErrorCallback != null) {
                connectionErrorCallback.execute();
            }
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    }
}