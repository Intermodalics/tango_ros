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
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NativeNodeMain;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

public class RunningActivity extends RosActivity implements TangoRosNode.CallbackListener {
    private static final String TAG = RunningActivity.class.getSimpleName();
    static final int START_SETTINGS_ACTIVITY_FIRST_RUN_REQUEST = 1;

    private SharedPreferences mSharedPref;
    private DrawerLayout mDrawerLayout;

    private TangoRosNode mTangoRosNode;
    private String mMasterUri = "";
    private ParameterNode mParameterNode;

    private boolean mIsTangoServiceBound = false;

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
                displayToastMessage(R.string.tango_bind_error);
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
            displayToastMessage( R.string.ros_init_error);
        } else if (errorCode < NativeNodeMain.SUCCESS) {
            Log.e(TAG, getResources().getString(R.string.tango_service_error));
            displayToastMessage(R.string.tango_service_error);
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.running_activity);
        mDrawerLayout = (DrawerLayout) findViewById(R.id.drawer_layout);
        getFragmentManager().beginTransaction().replace(R.id.preferencesFrame, new PrefsFragment()).commit();

        mSharedPref = PreferenceManager.getDefaultSharedPreferences(getBaseContext());
        mMasterUri = mSharedPref.getString(getString(R.string.pref_master_uri_key),
                getResources().getString(R.string.pref_master_uri_default));
        TextView uriTextView = (TextView) findViewById(R.id.master_uri);
        uriTextView.setText(mMasterUri);
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
                startSettingsActivity();
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
    protected void onDestroy() {
        super.onDestroy();
        if (mIsTangoServiceBound) {
            Log.i(TAG, "Unbind tango service");
            unbindService(mTangoServiceConnection);
        }
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
        if(TangoInitializationHelper.loadTangoSharedLibrary() !=
                TangoInitializationHelper.ARCH_ERROR) {
            mTangoRosNode = new TangoRosNode();
            mTangoRosNode.attachCallbackListener(this);
            TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
            if (mTangoRosNode.isTangoVersionOk(this)) {
                nodeMainExecutor.execute(mTangoRosNode, nodeConfiguration);
            } else {
                Log.e(TAG, getResources().getString(R.string.tango_version_error));
                displayToastMessage(R.string.tango_version_error);
            }
        } else {
            Log.e(TAG, getResources().getString(R.string.tango_lib_error));
            displayToastMessage(R.string.tango_lib_error);

        }
    }

    /**
     * This function is called when the NodeMainExecutorService is connected.
     * Overriding startMasterChooser allows to be sure that the NodeMainExecutorService is connected
     * when initializing and starting the node.
     */
    @Override
    public void startMasterChooser() {
        boolean appPreviouslyStarted = mSharedPref.getBoolean(getString(R.string.pref_previously_started_key), false);
        if (appPreviouslyStarted) {
            initAndStartRosJavaNode();
        } else {
            Intent intent = new Intent(this, SettingsActivity.class);
            startActivityForResult(intent, START_SETTINGS_ACTIVITY_FIRST_RUN_REQUEST);
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (requestCode == START_SETTINGS_ACTIVITY_FIRST_RUN_REQUEST) {
            if (resultCode == RESULT_CANCELED) { // Result code returned when back button is pressed.
                mMasterUri = mSharedPref.getString(getString(R.string.pref_master_uri_key),
                        getResources().getString(R.string.pref_master_uri_default));
                TextView uriTextView = (TextView) findViewById(R.id.master_uri);
                uriTextView.setText(mMasterUri);
                initAndStartRosJavaNode();
            }
        }
    }

    /**
     * This function initializes the tango ros node with RosJava interface.
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

    public void startSettingsActivity() {
        Intent intent = new Intent(this, SettingsActivity.class);
        startActivity(intent);
    }

    private void displayToastMessage(final int messageId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), messageId, Toast.LENGTH_SHORT).show();
            }
        });
    }
}