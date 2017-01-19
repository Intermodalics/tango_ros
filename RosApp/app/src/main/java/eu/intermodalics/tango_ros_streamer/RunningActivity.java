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

package eu.intermodalics.tango_ros_streamer;

import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.net.Uri;
import android.content.res.Configuration;
import android.os.AsyncTask;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.v4.widget.DrawerLayout;
import android.support.v7.widget.Toolbar;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import eu.intermodalics.tango_ros_node.TangoInitializationHelper;
import eu.intermodalics.tango_ros_node.TangoInitializationHelper.DefaultTangoServiceConnection;
import eu.intermodalics.tango_ros_node.TangoRosNode;

public class RunningActivity extends AppCompatRosActivity implements TangoRosNode.CallbackListener {
    private static final String TAG = RunningActivity.class.getSimpleName();
    private static final String TAGS_TO_LOG = TAG + ", " + "tango_client_api, " + "Registrar, "
            + "DefaultPublisher, " + "native, " + "DefaultPublisher" ;
    private static final int LOG_TEXT_MAX_LENGTH = 5000;

    public static class startSettingsActivityRequest {
        public static final int FIRST_RUN = 1;
        public static final int STANDARD_RUN = 2;
    }
    enum RosStatus {
        MASTER_NOT_CONNECTED,
        NODE_RUNNING
    }

    enum TangoStatus {
        SERVICE_NOT_BOUND,
        SERVICE_NOT_CONNECTED,
        VERSION_NOT_SUPPORTED,
        SERVICE_RUNNING,
    }

    private SharedPreferences mSharedPref;
    private TangoRosNode mTangoRosNode;
    private String mMasterUri = "";
    private ParameterNode mParameterNode;
    private RosStatus mRosStatus = RosStatus.MASTER_NOT_CONNECTED;
    private TangoStatus mTangoStatus = TangoStatus.SERVICE_NOT_CONNECTED;
    private Logger mLogger;

    // UI objects.
    private DrawerLayout mDrawerLayout;
    private TextView mUriTextView;
    private ImageView mRosLightImageView;
    private ImageView mTangoLightImageView;
    private Switch mlogSwitch;
    private boolean mDisplayLog = false;
    private TextView mLogTextView;

    public RunningActivity() {
        super("TangoxRos", "TangoxRos");
    }

    protected RunningActivity(String notificationTicker, String notificationTitle) {
        super(notificationTicker, notificationTitle);
    }

    /**
     * Tango Service connection.
     */
    ServiceConnection mTangoServiceConnection = new DefaultTangoServiceConnection(
        new DefaultTangoServiceConnection.AfterConnectionCallback() {
            @Override
            public void execute() {
                if (TangoInitializationHelper.isTangoServiceBound()) {
                    if (TangoInitializationHelper.isTangoVersionOk()) {
                        updateTangoStatus(TangoStatus.SERVICE_RUNNING);
                    } else {
                        updateTangoStatus(TangoStatus.VERSION_NOT_SUPPORTED);
                    }
                } else {
                    updateTangoStatus(TangoStatus.SERVICE_NOT_BOUND);
                    Log.e(TAG, getString(R.string.tango_bind_error));
                    displayToastMessage(R.string.tango_bind_error);
                    onDestroy();
                }
            }
        }
    );

    /**
     * Implements TangoRosNode.CallbackListener.
     */
    public void onTangoRosErrorHook(int returnCode) {
        if (returnCode == TangoRosNode.ROS_CONNECTION_ERROR) {
            updateRosStatus(RosStatus.MASTER_NOT_CONNECTED);
            Log.e(TAG, getString(R.string.ros_init_error));
            displayToastMessage(R.string.ros_init_error);
        } else if (returnCode < TangoRosNode.SUCCESS) {
            updateTangoStatus(TangoStatus.SERVICE_NOT_CONNECTED);
            Log.e(TAG, getString(R.string.tango_service_error));
            displayToastMessage(R.string.tango_service_error);
        }
    }

    private void updateRosStatus(RosStatus status) {
        if (mRosStatus != status) {
            mRosStatus = status;
            switchRosLight(status);
        }
    }

    private void switchRosLight(final RosStatus status) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (status == RosStatus.NODE_RUNNING) {
                    mRosLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_green_light));
                } else {
                    mRosLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_red_light));
                }
            }
        });
    }

    private void updateTangoStatus(TangoStatus status) {
        if (mTangoStatus != status) {
            mTangoStatus = status;
            switchTangoLight(status);
        }
    }

    private void switchTangoLight(final TangoStatus status) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (status == TangoStatus.SERVICE_RUNNING) {
                    mTangoLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_green_light));
                } else {
                    mTangoLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_red_light));
                }
            }
        });
    }

    /**
     * Display a toast message with the given message.
     * @param messageId String id of the message to display.
     */
    private void displayToastMessage(final int messageId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), messageId, Toast.LENGTH_SHORT).show();
            }
        });
    }

    private void setupUI() {
        setContentView(R.layout.running_activity);
        mDrawerLayout = (DrawerLayout) findViewById(R.id.drawer_layout);
        getFragmentManager().beginTransaction().replace(R.id.preferencesFrame, new PrefsFragment()).commit();
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        mUriTextView = (TextView) findViewById(R.id.master_uri);
        mUriTextView.setText(mMasterUri);
        mRosLightImageView = (ImageView) findViewById(R.id.is_ros_ok_image);
        mTangoLightImageView = (ImageView) findViewById(R.id.is_tango_ok_image);
        mlogSwitch = (Switch) findViewById(R.id.log_switch);
        mlogSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean isChecked) {
                mDisplayLog = isChecked;
                mLogTextView.setVisibility(isChecked ? View.VISIBLE : View.INVISIBLE);
            }
        });
        mLogTextView = (TextView)findViewById(R.id.log_view);
        mLogTextView.setMovementMethod(new ScrollingMovementMethod());
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        mSharedPref = PreferenceManager.getDefaultSharedPreferences(getBaseContext());
        mMasterUri = mSharedPref.getString(getString(R.string.pref_master_uri_key),
                getResources().getString(R.string.pref_master_uri_default));
        String logFileName = mSharedPref.getString(getString(R.string.pref_log_file_key),
                getString(R.string.pref_log_file_default));
        setupUI();
        mLogger = new Logger(this, mLogTextView, TAGS_TO_LOG, logFileName, LOG_TEXT_MAX_LENGTH);
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        setupUI();
        switchRosLight(mRosStatus);
        switchTangoLight(mTangoStatus);
        mlogSwitch.setChecked(mDisplayLog);
        mLogTextView.setText(mLogger.getLogText());
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
                Intent settingsActivityIntent = new Intent(this, SettingsActivity.class);
                startActivityForResult(settingsActivityIntent, startSettingsActivityRequest.STANDARD_RUN);
                return true;
            case R.id.drawer:
                if (mDrawerLayout.isDrawerOpen(Gravity.RIGHT)) {
                    mDrawerLayout.closeDrawer(Gravity.RIGHT);
                } else {
                    mDrawerLayout.openDrawer(Gravity.RIGHT);
                }
                return true;
            case R.id.share:
                mLogger.saveLogToFile();
                Intent shareFileIntent = new Intent(Intent.ACTION_SEND);
                shareFileIntent.setType("text/plain");
                shareFileIntent.putExtra(Intent.EXTRA_STREAM, Uri.fromFile(mLogger.getLogFile()));
                startActivity(shareFileIntent);
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (TangoInitializationHelper.isTangoServiceBound()) {
            Log.i(TAG, "Unbind tango service");
            unbindService(mTangoServiceConnection);
            updateTangoStatus(TangoStatus.SERVICE_NOT_BOUND);
        }
        mLogger.saveLogToFile();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (resultCode == RESULT_CANCELED) { // Result code returned when back button is pressed.
            if (requestCode == startSettingsActivityRequest.FIRST_RUN) {
                mMasterUri = mSharedPref.getString(getString(R.string.pref_master_uri_key),
                        getResources().getString(R.string.pref_master_uri_default));
                mUriTextView.setText(mMasterUri);
                String logFileName = mSharedPref.getString(getString(R.string.pref_log_file_key),
                        getString(R.string.pref_log_file_default));
                mLogger.setLogFileName(logFileName);
                mLogger.start();
                initAndStartRosJavaNode();
            } else if (requestCode == startSettingsActivityRequest.STANDARD_RUN) {
                // It is ok to change the log file name at runtime.
                String logFileName = mSharedPref.getString(getString(R.string.pref_log_file_key),
                        getString(R.string.pref_log_file_default));
                mLogger.setLogFileName(logFileName);
            }
        }
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(this.nodeMainExecutorService.getMasterUri());

        // Create parameter synchronization node to be up-to-date with Dynamic Reconfigure.
        mParameterNode = new ParameterNode(this,
                getString(R.string.publish_device_pose_key),
                getString(R.string.publish_point_cloud_key),
                getString(R.string.publish_color_camera_key),
                getString(R.string.publish_fisheye_camera_key));
        nodeConfiguration.setNodeName(mParameterNode.getDefaultNodeName());
        nodeMainExecutor.execute(mParameterNode, nodeConfiguration);

        // Create and start Tango ROS Node
        nodeConfiguration.setNodeName(TangoRosNode.NODE_NAME);
        if (TangoInitializationHelper.loadTangoSharedLibrary() !=
                TangoInitializationHelper.ARCH_ERROR) {
            mTangoRosNode = new TangoRosNode();
            mTangoRosNode.attachCallbackListener(this);
            TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
            if (TangoInitializationHelper.checkTangoVersionOk(this)) {
                nodeMainExecutor.execute(mTangoRosNode, nodeConfiguration);
                updateRosStatus(RosStatus.NODE_RUNNING);
            } else {
                updateTangoStatus(TangoStatus.VERSION_NOT_SUPPORTED);
                Log.e(TAG, getResources().getString(R.string.tango_version_error));
                displayToastMessage(R.string.tango_version_error);
            }
        } else {
            Log.e(TAG, getString(R.string.tango_lib_error));
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
            mLogger.start();
            initAndStartRosJavaNode();
        } else {
            Intent intent = new Intent(this, SettingsActivity.class);
            startActivityForResult(intent, startSettingsActivityRequest.FIRST_RUN);
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
}