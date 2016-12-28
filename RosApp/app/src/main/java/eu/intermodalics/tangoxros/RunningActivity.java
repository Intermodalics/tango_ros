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

package eu.intermodalics.tangoxros;

import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.IBinder;
import android.preference.PreferenceManager;
import android.support.v4.widget.DrawerLayout;
import android.util.Log;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NativeNodeMain;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URI;

public class RunningActivity extends RosActivity implements TangoRosNode.CallbackListener {
    private static final String TAG = RunningActivity.class.getSimpleName();

    private SharedPreferences mSharedPref;
    private DrawerLayout mDrawerLayout;

    private TangoRosNode mTangoRosNode;
    private String mMasterUri = "";
    private ParameterNode mParameterNode = null;
    private PrefsFragment mPrefsFragment = null;
    private PublisherConfiguration mPublishConfig = new PublisherConfiguration();

    private boolean mIsNodeStarted = true;
    private boolean mIsNodeRunning = false;
    private boolean mIsTangoServiceBound = false;

    private TextView mLogTextView;
    private Thread logThread;
    private StringBuilder mLogStringBuilder;

    public RunningActivity() {
        super("TangoxRos", "TangoxRos");
    }

    protected RunningActivity(String notificationTicker, String notificationTitle) {
        super(notificationTicker, notificationTitle);
    }

    /**
     * Tango Service connection.
     */
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            // Synchronization around RunningActivity object is to avoid
            // Tango disconnect in the middle of the connecting operation.
            if (mTangoRosNode.setBinderTangoService(service)) {
                Log.i(TAG, "Bound to tango service");
                mIsTangoServiceBound = true;
            } else {
                Log.e(TAG, getResources().getString(R.string.tango_bind_error));
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(getApplicationContext(), R.string.tango_bind_error, Toast.LENGTH_SHORT).show();
                    }
                });
                onDestroy();
            }
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };

    /**
     * Implements TangoRosNode.CallbackListener.
     */
    public void onNativeNodeExecutionError(int errorCode) {
        if (errorCode == NativeNodeMain.ROS_CONNECTION_ERROR) {
            Log.e(TAG, getResources().getString(R.string.ros_init_error));
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(getApplicationContext(), R.string.ros_init_error, Toast.LENGTH_SHORT).show();
                }
            });
        } else if (errorCode < NativeNodeMain.SUCCESS) {
            Log.e(TAG, getResources().getString(R.string.tango_service_error));
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(getApplicationContext(), R.string.tango_service_error, Toast.LENGTH_SHORT).show();
                }
            });
        }
    }

    private void updateLogTextView() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mLogTextView.setText(mLogStringBuilder.toString());
            }
        });
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.running_activity);
        mDrawerLayout = (DrawerLayout) findViewById(R.id.drawer_layout);
        mDrawerLayout.setDrawerListener(new DrawerLayout.DrawerListener() {
            @Override
            public void onDrawerSlide(View drawerView, float slideOffset) {
            }

            @Override
            public void onDrawerOpened(View drawerView) {
            }

            @Override
            public void onDrawerClosed(View drawerView) {
                applySettings();
            }

            @Override
            public void onDrawerStateChanged(int newState) {
            }
        });
        getFragmentManager().beginTransaction().replace(R.id.preferencesFrame, new PrefsFragment()).commit();

        mLogTextView = (TextView)findViewById(R.id.log_view);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.settings:
                runSettingsActivity();
                return true;
            case R.id.drawer:
                if (mDrawerLayout.isDrawerOpen(Gravity.RIGHT)) {
                    mDrawerLayout.closeDrawer(Gravity.RIGHT);
                } else {
                    mDrawerLayout.openDrawer(Gravity.RIGHT);
                }
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    @Override
    protected void onStart() {
        super.onStart();
        mPrefsFragment = (PrefsFragment) getFragmentManager().findFragmentById(R.id.preferencesFrame);
        mSharedPref = PreferenceManager.getDefaultSharedPreferences(getBaseContext());

        boolean appPreviouslyStarted = mSharedPref.getBoolean(getString(R.string.pref_previously_started_key), false);
        if(!appPreviouslyStarted) {
            runSettingsActivity();
            mIsNodeStarted = false;
        } else {
            // Avoid changing master URI while node is still running.
            if (!mIsNodeRunning) {
                mMasterUri = mSharedPref.getString(getString(R.string.pref_master_uri_key),
                        getResources().getString(R.string.pref_master_uri_default));
                TextView uriTextView;
                uriTextView = (TextView) findViewById(R.id.master_uri);
                uriTextView.setText(mMasterUri);
            }
            // Avoid restarting the node if it was already started.
            if (!mIsNodeStarted) {
                initAndStartRosJavaNode();
                mIsNodeStarted = true;
            }
        }
        final String cmd = "logcat -d -s " + TAG + ", tango_client_api";
        logThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    try {
                        Process process = Runtime.getRuntime().exec(cmd);
                        BufferedReader bufferedReader = new BufferedReader(
                                new InputStreamReader(process.getInputStream()));
                        String line = "";
                        mLogStringBuilder = new StringBuilder();
                        while ((line = bufferedReader.readLine()) != null) {
                            mLogStringBuilder.append(line + '\n');
                        }
                        mLogStringBuilder.reverse();
                        mLogStringBuilder.setLength(2000);
                        mLogStringBuilder.reverse();
                        updateLogTextView();
                        logThread.sleep(200);
                    } catch (IOException e) {
                        Log.e(TAG, e.toString());
                    } catch (InterruptedException e) {
                        Log.e(TAG, e.toString());
                    }
                }
            }
        });
        logThread.start();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (mIsTangoServiceBound) {
            Log.i(TAG, "Unbind tango service");
            unbindService(mTangoServiceConnection);
        }
    }

    // This function shall be removed once Dynamic Reconfigure is implemented on the Java side of the app.
    public void applySettings() {
        mPublishConfig = mPrefsFragment.getPublisherConfigurationFromPreferences();
        mTangoRosNode.updatePublisherConfiguration(mPublishConfig);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(this.nodeMainExecutorService.getMasterUri());

        // Create parameter synchronization node to be up-to-date with Parameter Server.
        mParameterNode = new ParameterNode(this,
                getString(R.string.publish_device_pose_key),
                getString(R.string.publish_point_cloud_key),
                getString(R.string.publish_color_camera_key),
                getString(R.string.publish_fisheye_camera_key));
        nodeConfiguration.setNodeName(mParameterNode.getDefaultNodeName());
        nodeMainExecutor.execute(mParameterNode, nodeConfiguration);

        // Create and start Tango ROS Node
        nodeConfiguration.setNodeName(TangoRosNode.NODE_NAME);
        mTangoRosNode = new TangoRosNode();
        mTangoRosNode.attachCallbackListener(this);
        TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
        if (mTangoRosNode.isTangoVersionOk(this)) {
            mIsNodeRunning = true;
            nodeMainExecutor.execute(mTangoRosNode, nodeConfiguration);
        } else {
            Log.e(TAG, getResources().getString(R.string.tango_version_error));
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(getApplicationContext(), R.string.tango_version_error, Toast.LENGTH_SHORT).show();
                }
            });
        }
    }

    /**
     * This function allows initialization of the node with RosJava interface without using MasterChooser,
     * and is compatible with current Master Uri setter interface.
     */
    private void initAndStartRosJavaNode() {
        if (mMasterUri != null) {
            URI masterUri;
            try {
                masterUri = URI.create(mMasterUri);
            } catch (IllegalArgumentException e) {
                Log.e(TAG, "Wrong URI: " + e.getMessage());
                return;
            }
            this.nodeMainExecutorService.setMasterUri(masterUri);
            new AsyncTask<Void, Void, Void>() {
                @Override
                protected Void doInBackground(Void... params) {
                    RunningActivity.this.init(nodeMainExecutorService);
                    return null;
                }
            }.execute();
        } else {
            Log.e(TAG, "Master URI is null");
        }
    }

    /**
     * Override startMasterChooser to be sure that the node main executor service is connected
     * when initializing and starting the node.
     */
    @Override
    public void startMasterChooser() {
        boolean appPreviouslyStarted = mSharedPref.getBoolean(getString(R.string.pref_previously_started_key), false);
        if (appPreviouslyStarted) {
            initAndStartRosJavaNode();
        }
    }

    public void runSettingsActivity() {
        Intent intent = new Intent(this, SettingsActivity.class);
        startActivity(intent);
    }
}