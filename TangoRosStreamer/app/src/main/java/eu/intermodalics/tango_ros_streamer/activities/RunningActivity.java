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

package eu.intermodalics.tango_ros_streamer.activities;

import android.animation.AnimatorInflater;
import android.animation.AnimatorSet;
import android.app.DialogFragment;
import android.app.FragmentManager;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.net.Uri;
import android.content.res.Configuration;
import android.net.wifi.WifiManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.StrictMode;
import android.preference.PreferenceManager;
import android.support.design.widget.Snackbar;
import android.support.v7.widget.Toolbar;
import android.text.format.Formatter;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import org.apache.commons.io.FilenameUtils;
import org.ros.address.InetAddressFactory;
import org.ros.android.NodeMainExecutorService;
import org.ros.android.NodeMainExecutorServiceListener;
import org.ros.exception.RosRuntimeException;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeListener;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeListener;
import org.ros.node.NodeMainExecutor;

import java.io.File;
import java.net.URI;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.CountDownLatch;

import eu.intermodalics.nodelet_manager.TangoNodeletManager;
import eu.intermodalics.nodelet_manager.TangoInitializationHelper;
import eu.intermodalics.nodelet_manager.TangoInitializationHelper.DefaultTangoServiceConnection;

import eu.intermodalics.tango_ros_common.Logger;
import eu.intermodalics.tango_ros_common.MasterConnectionChecker;
import eu.intermodalics.tango_ros_common.TangoServiceClientNode;
import eu.intermodalics.tango_ros_streamer.android.LoadOccupancyGridDialog;
import eu.intermodalics.tango_ros_streamer.nodes.ImuNode;
import eu.intermodalics.tango_ros_common.ParameterNode;
import eu.intermodalics.tango_ros_streamer.R;
import eu.intermodalics.tango_ros_streamer.android.SaveMapDialog;
import tango_ros_messages.TangoConnectRequest;
import tango_ros_messages.TangoConnectResponse;

public class RunningActivity extends AppCompatRosActivity implements
        SaveMapDialog.CallbackListener, LoadOccupancyGridDialog.CallbackListener,
        TangoServiceClientNode.CallbackListener {
    private static final String TAG = RunningActivity.class.getSimpleName();
    private static final String TAGS_TO_LOG = TAG + ", " + "tango_client_api, " + "Registrar, "
            + "DefaultPublisher, " + "native, " + "DefaultPublisher" ;
    private static final int LOG_TEXT_MAX_LENGTH = 5000;
    private static final int MAX_TANGO_CONNECTION_TRY = 50;

    private static final String REQUEST_TANGO_PERMISSION_ACTION = "android.intent.action.REQUEST_TANGO_PERMISSION";
    public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";
    public static final String EXTRA_VALUE_ADF = "ADF_LOAD_SAVE_PERMISSION";
    private static final String EXTRA_VALUE_DATASET = "DATASET_PERMISSION";
    private static final int REQUEST_CODE_ADF_PERMISSION = 111;
    private static final int REQUEST_CODE_DATASET_PERMISSION = 112;
    public static final String RESTART_TANGO = "restart_tango";

    public static class StartSettingsActivityRequest {
        public static final int FIRST_RUN = 11;
        public static final int STANDARD_RUN = 12;
    }

    enum RosStatus {
        UNKNOWN,
        MASTER_NOT_CONNECTED,
        MASTER_CONNECTED
    }

    // Symmetric implementation to tango_ros_node.h.
    enum TangoStatus {
        UNKNOWN,
        SERVICE_NOT_CONNECTED,
        NO_FIRST_VALID_POSE,
        SERVICE_CONNECTED
    }

    private SharedPreferences mSharedPref;
    private TangoNodeletManager mTangoNodeletManager;
    private boolean mRunLocalMaster = false;
    private String mMasterUri = "";
    private CountDownLatch mRosConnectionLatch;
    private ParameterNode mParameterNode;
    private TangoServiceClientNode mTangoServiceClientNode;
    private ImuNode mImuNode;
    private RosStatus mRosStatus = RosStatus.UNKNOWN;
    private TangoStatus mTangoStatus = TangoStatus.UNKNOWN;
    private Logger mLogger;
    private boolean mCreateNewMap = false;
    private boolean mMapSaved = false;
    private HashMap<String, String> mUuidsNamesHashMap;
    // True after the user answered the ADF permission popup (the permission has not been necessarily granted).
    private boolean mAdfPermissionHasBeenAnswered = false;
    // True after the user answered the dataset permission popup (the permission has not been necessarily granted).
    private boolean mDatasetPermissionHasBeenAnswered = false;
    private ArrayList<String> mOccupancyGridNameList = new ArrayList<String>();

    // UI objects.
    private Menu mToolbarMenu;
    private TextView mUriTextView;
    private ImageView mRosLightImageView;
    private ImageView mTangoLightImageView;
    private Switch mlogSwitch;
    private boolean mDisplayLog = false;
    private TextView mLogTextView;
    private Button mSaveMapButton;
    private Button mLoadOccupancyGridButton;
    private Snackbar mSnackbarLoadNewMap;
    private Snackbar mSnackbarRosReconnection;

    public RunningActivity() {
        super("TangoRosStreamer", "TangoRosStreamer");
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
                        Log.i(TAG, "Version of Tango is ok.");
                    } else {
                        updateTangoStatus(TangoStatus.SERVICE_NOT_CONNECTED);
                        Log.e(TAG, getResources().getString(R.string.tango_version_error));
                        displayToastMessage(R.string.tango_version_error);
                    }
                } else {
                    updateTangoStatus(TangoStatus.SERVICE_NOT_CONNECTED);
                    Log.e(TAG, getString(R.string.tango_bind_error));
                    displayToastMessage(R.string.tango_bind_error);
                }
            }
        }
    );

    private void updateRosStatus(RosStatus status) {
        if (mRosStatus != status) {
            mRosStatus = status;
        }
        switchRosLight(status);
        SharedPreferences.Editor editor = mSharedPref.edit();
        editor.putInt(getString(R.string.ros_status), status.ordinal());
        editor.commit();
    }

    private void switchRosLight(final RosStatus status) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (status == RosStatus.UNKNOWN) {
                    mRosLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_orange_light));
                } else if (status == RosStatus.MASTER_CONNECTED) {
                    // Turn ROS light to green.
                    mRosLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_green_light));
                    // Dismiss ROS reconnection snackbar if necessary.
                    if (mSnackbarRosReconnection != null && mSnackbarRosReconnection.isShown()) {
                        mSnackbarRosReconnection.dismiss();
                    }
                    // Set settings icon color to white.
                    mToolbarMenu.findItem(R.id.settings).setIcon(R.drawable.ic_settings_white_24dp);
                } else if (status == RosStatus.MASTER_NOT_CONNECTED) {
                    // Turn ROS light to red.
                    mRosLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_red_light));
                    // Show snackbar with ROS reconnection button.
                    mSnackbarRosReconnection = Snackbar.make(findViewById(android.R.id.content),
                            getString(R.string.snackbar_text_reconnect_ros), Snackbar.LENGTH_INDEFINITE);
                    mSnackbarRosReconnection.setAction(getString(R.string.snackbar_action_text_reconnect_ros),
                            new View.OnClickListener() {
                                @Override
                                public void onClick(View view) {
                                    mRunLocalMaster = mSharedPref.getBoolean(getString(R.string.pref_master_is_local_key), false);
                                    mMasterUri = mSharedPref.getString(getString(R.string.pref_master_uri_key),
                                            getResources().getString(R.string.pref_master_uri_default));
                                    mUriTextView.setText(mMasterUri);
                                    initAndStartRosJavaNode();
                                }
                            }
                    );
                    View snackBarView = mSnackbarRosReconnection.getView();
                    snackBarView.setBackgroundColor(getResources().getColor(android.R.color.holo_red_dark));
                    mSnackbarRosReconnection.show();
                    // Highlight ROS Master URI.
                    AnimatorSet set = (AnimatorSet) AnimatorInflater.loadAnimator(RunningActivity.this, R.animator.master_uri_text_animation);
                    set.setTarget(mUriTextView);
                    set.start();
                    // Set settings icon color to red.
                    mToolbarMenu.findItem(R.id.settings).setIcon(R.drawable.ic_settings_red_24dp);
                }
            }
        });
    }

    private void updateTangoStatus(TangoStatus status) {
        if (mTangoStatus != status) {
            mTangoStatus = status;
            switchTangoLight(status);
            if (status == TangoStatus.NO_FIRST_VALID_POSE) {
                displayToastMessage(R.string.point_device);
            }
        }
    }

    private void switchTangoLight(final TangoStatus status) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (status == TangoStatus.UNKNOWN) {
                    mTangoLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_orange_light));
                } else if (status == TangoStatus.SERVICE_CONNECTED) {
                    mTangoLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_green_light));
                } else {
                    mTangoLightImageView.setImageDrawable(getResources().getDrawable(R.drawable.btn_radio_on_red_light));
                }
            }
        });
    }

    private void updateLoadAndSaveMapButtons() {
        mCreateNewMap = mSharedPref.getBoolean(getString(R.string.pref_create_new_map_key), false);
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mSaveMapButton.setEnabled(!mMapSaved);
                if (mCreateNewMap) {
                    mSaveMapButton.setVisibility(View.VISIBLE);
                    mLoadOccupancyGridButton.setVisibility(View.GONE);
                } else {
                    mSaveMapButton.setVisibility(View.GONE);
                    mLoadOccupancyGridButton.setVisibility(View.VISIBLE);
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
                Toast.makeText(getApplicationContext(), messageId, Toast.LENGTH_LONG).show();
            }
        });
    }

    private void showSaveMapDialog() {
        FragmentManager manager = getFragmentManager();
        SaveMapDialog saveMapDialog = new SaveMapDialog();
        saveMapDialog.setStyle(DialogFragment.STYLE_NORMAL, R.style.CustomDialog);
        saveMapDialog.show(manager, "SaveMapDialog");
    }

    private void showLoadOccupancyGridDialog(boolean firstTry, java.util.ArrayList<java.lang.String> nameList) {
        FragmentManager manager = getFragmentManager();
        LoadOccupancyGridDialog loadOccupancyGridDialog = new LoadOccupancyGridDialog();
        Bundle bundle = new Bundle();
        bundle.putBoolean(getString(R.string.show_load_occupancy_grid_empty_key), nameList.isEmpty());
        bundle.putBoolean(getString(R.string.show_load_occupancy_grid_error_key), !firstTry);
        bundle.putStringArrayList(getString(R.string.list_names_occupancy_grid_key), nameList);
        loadOccupancyGridDialog.setArguments(bundle);
        loadOccupancyGridDialog.setStyle(DialogFragment.STYLE_NORMAL, R.style.CustomDialog);
        loadOccupancyGridDialog.show(manager, "LoadOccupancyGridDialog");
    }

    private void setupUI() {
        setContentView(R.layout.running_activity);
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
        mSaveMapButton = (Button) findViewById(R.id.save_map_button);
        mSaveMapButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showSaveMapDialog();
            }
        });
        mLoadOccupancyGridButton = (Button) findViewById(R.id.load_occupancy_grid_button);
        mLoadOccupancyGridButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                mOccupancyGridNameList = new ArrayList<String>();
                try {
                    String directory = mParameterNode.getStringParam(getString(R.string.occupancy_grid_directory_key));
                    File occupancyGridDirectory = new File(directory);
                    if (occupancyGridDirectory != null && occupancyGridDirectory.isDirectory()) {
                        File[] files = occupancyGridDirectory.listFiles();
                        for (File file : files) {
                            if (FilenameUtils.getExtension(file.getName()).equals("yaml")) {
                                mOccupancyGridNameList.add(FilenameUtils.removeExtension(file.getName()));
                            }
                        }
                    }
                    showLoadOccupancyGridDialog(/* firstTry */ true, mOccupancyGridNameList);
                } catch (RuntimeException e) {
                    e.printStackTrace();
                }
            }
        });
        updateLoadAndSaveMapButtons();
    }

    public void onClickOkSaveMapDialog(final String mapName) {
        assert(mapName !=null && !mapName.isEmpty());
        mSaveMapButton.setEnabled(false);
        new AsyncTask<Void, Void, Void>() {
            @Override
            protected Void doInBackground(Void... params) {
                mTangoServiceClientNode.callSaveMapService(mapName);
                return null;
            }
        }.execute();
    }

    @Override
    public void onSaveMapServiceCallFinish(boolean success, final String message,
                                           final String mapName, final String mapUuid) {
        if (success) {
            mMapSaved = true;
            displayToastMessage(R.string.save_map_success);
            saveUuidsNamestoHashMap();
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mSaveMapButton.setEnabled(!mMapSaved);
                    mSnackbarLoadNewMap = Snackbar.make(findViewById(android.R.id.content),
                            getString(R.string.snackbar_text_load_new_map), Snackbar.LENGTH_INDEFINITE);
                    mSnackbarLoadNewMap.setAction(getString(R.string.snackbar_action_text_load_new_map), new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            try {
                                mParameterNode.changeSettingsToLocalizeInMap(mapUuid, getString(R.string.pref_create_new_map_key),
                                        getString(R.string.pref_localization_mode_key), getString(R.string.pref_localization_map_uuid_key));
                                restartTango();
                            } catch (RuntimeException e) {
                                e.printStackTrace();
                            }
                        }
                    });
                    mSnackbarLoadNewMap.show();
                }
            });
        } else {
            Log.e(TAG, "Error while saving map: " + message);
            displayToastMessage(R.string.save_map_error);
        }
    }

    public void onClickItemLoadOccupancyGridDialog(final String occupancyGridName) {
        assert(occupancyGridName !=null && !occupancyGridName.isEmpty());
        mLoadOccupancyGridButton.setEnabled(false);
        new AsyncTask<Void, Void, Void>() {
            @Override
            protected Void doInBackground(Void... params) {
                mTangoServiceClientNode.callLoadOccupancyGridService(occupancyGridName);
                return null;
            }
        }.execute();
    }

    @Override
    public void onLoadOccupancyGridServiceCallFinish(boolean success, final String message,
                                                     boolean aligned, String mapUuid) {
        if (success) {
            if (aligned) {
                displayToastMessage(R.string.load_occupancy_grid_success);
            } else {
                displayToastMessage(R.string.load_occupancy_grid_not_aligned);
            }
            Log.i(TAG, message);
        } else {
            Log.e(TAG, "Error while loading occupancy grid: " + message);
            displayToastMessage(R.string.load_occupancy_grid_error);
            showLoadOccupancyGridDialog(/* firstTry */ false, mOccupancyGridNameList);
        }
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mLoadOccupancyGridButton.setEnabled(true);
            }
        });
    }

    @Override
    public void onTangoConnectServiceFinish(int response, String message) {
        if (response != TangoConnectResponse.TANGO_SUCCESS) {
            Log.e(TAG, "Error connecting to Tango: " + response + ", message: " + message);
            displayToastMessage(R.string.tango_connect_error);
            return;
        }
        displayToastMessage(R.string.tango_connect_success);
    }

    @Override
    public void onTangoDisconnectServiceFinish(int response, String message) {
        if (response != TangoConnectResponse.TANGO_SUCCESS) {
            Log.e(TAG, "Error disconnecting from Tango: " + response + ", message: " + message);
            // Do not switch Tango lights in this case because Tango disconnect can never fail.
            // Failure occured due to something else, so Tango is still connected.
            displayToastMessage(R.string.tango_disconnect_error);
            return;
        }
        displayToastMessage(R.string.tango_disconnect_success);
    }

    @Override
    public void onTangoReconnectServiceFinish(int response, String message) {
        if (response != TangoConnectResponse.TANGO_SUCCESS) {
            Log.e(TAG, "Error reconnecting to Tango: " + response + ", message: " + message);
            displayToastMessage(R.string.tango_reconnect_error);
            return;
        }
        displayToastMessage(R.string.tango_reconnect_success);
    }

    public void onGetMapUuidsFinish(List<String> mapUuids, List<String> mapNames) {
        mUuidsNamesHashMap = new HashMap<>();
        if (mapUuids == null || mapNames == null) return;
        assert(mapUuids.size() == mapNames.size());
        for (int i = 0; i < mapUuids.size(); ++i) {
            mUuidsNamesHashMap.put(mapUuids.get(i), mapNames.get(i));
        }
        if(mParameterNode != null) {
            try {
                mParameterNode.setPreferencesFromParameterServer();
            } catch (RuntimeException e) {
                e.printStackTrace();
            }
        }
        Intent settingsActivityIntent = new Intent(SettingsActivity.NEW_UUIDS_NAMES_MAP_ALERT);
        settingsActivityIntent.putExtra(getString(R.string.uuids_names_map), mUuidsNamesHashMap);
        this.sendBroadcast(settingsActivityIntent);
    }

    @Override
    public void onTangoStatus(int status) {
        if (status >= TangoStatus.values().length) {
            Log.e(TAG, "Invalid Tango status " + status);
            return;
        }
        if (status == TangoStatus.SERVICE_CONNECTED.ordinal() && mTangoStatus != TangoStatus.SERVICE_CONNECTED) {
            saveUuidsNamestoHashMap();
            try {
                mParameterNode.setPreferencesFromParameterServer();
            } catch (RuntimeException e) {
                e.printStackTrace();
            }
            mMapSaved = false;
            if (mSnackbarLoadNewMap != null && mSnackbarLoadNewMap.isShown()) {
                mSnackbarLoadNewMap.dismiss();
            }
        }
        updateLoadAndSaveMapButtons();
        updateTangoStatus(TangoStatus.values()[status]);
    }

    private void saveUuidsNamestoHashMap() {
        mTangoServiceClientNode.callGetMapUuidsService();
    }

    private void getTangoPermission(String permissionType, int requestCode) {
        Intent intent = new Intent();
        intent.setAction(REQUEST_TANGO_PERMISSION_ACTION);
        intent.putExtra(EXTRA_KEY_PERMISSIONTYPE, permissionType);
        startActivityForResult(intent, requestCode);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Thread.setDefaultUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
            public void uncaughtException(Thread t, Throwable e) {
                Log.e(TAG, "Uncaught exception of type " + e.getClass());
                e.printStackTrace();
            }
        });
        // The following piece of code allows networking in main thread.
        // e.g. Restart Tango button calls a ROS service in UI thread.
        if (android.os.Build.VERSION.SDK_INT > 9) {
            StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
            StrictMode.setThreadPolicy(policy);
        }
        mSharedPref = PreferenceManager.getDefaultSharedPreferences(getBaseContext());
        mRunLocalMaster = mSharedPref.getBoolean(getString(R.string.pref_master_is_local_key), false);
        mMasterUri = mSharedPref.getString(getString(R.string.pref_master_uri_key),
                getResources().getString(R.string.pref_master_uri_default));
        mCreateNewMap = mSharedPref.getBoolean(getString(R.string.pref_create_new_map_key), false);
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
        mToolbarMenu = menu;
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.settings:
                if(mParameterNode != null) {
                    try {
                        mParameterNode.setPreferencesFromParameterServer();
                    } catch (RuntimeException e) {
                        e.printStackTrace();
                    }
                }
                Intent settingsActivityIntent = new Intent(this, SettingsActivity.class);
                settingsActivityIntent.putExtra(getString(R.string.uuids_names_map), mUuidsNamesHashMap);
                startActivityForResult(settingsActivityIntent, StartSettingsActivityRequest.STANDARD_RUN);
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

    private void unbindFromTango() {
        if (TangoInitializationHelper.isTangoServiceBound()) {
            Log.i(TAG, "Unbind tango service");
            TangoInitializationHelper.unbindTangoService(this, mTangoServiceConnection);
            updateTangoStatus(TangoStatus.SERVICE_NOT_CONNECTED);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (mParameterNode != null) {
            try {
                mParameterNode.setPreferencesFromParameterServer();
            } catch (RuntimeException e) {
                e.printStackTrace();
            }
        }
        this.nodeMainExecutorService.forceShutdown();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (resultCode == RESULT_CANCELED) { // Result code returned when back button is pressed.
            // Upload new settings to parameter server.
            if ((requestCode == StartSettingsActivityRequest.STANDARD_RUN ||
                    requestCode == StartSettingsActivityRequest.FIRST_RUN) &&
                    mParameterNode != null) {
                try {
                    mParameterNode.uploadPreferencesToParameterServer();
                } catch (RuntimeException e) {
                    e.printStackTrace();
                }
            }

            if (data != null && data.getBooleanExtra(RESTART_TANGO, false)) {
                restartTango();
            }

            if (requestCode == StartSettingsActivityRequest.FIRST_RUN) {
                mRunLocalMaster = mSharedPref.getBoolean(getString(R.string.pref_master_is_local_key), false);
                mMasterUri = mSharedPref.getString(getString(R.string.pref_master_uri_key),
                        getResources().getString(R.string.pref_master_uri_default));
                mUriTextView.setText(mMasterUri);
                String logFileName = mSharedPref.getString(getString(R.string.pref_log_file_key),
                        getString(R.string.pref_log_file_default));
                mLogger.setLogFileName(logFileName);
                mLogger.start();
                getTangoPermission(EXTRA_VALUE_ADF, REQUEST_CODE_ADF_PERMISSION);
                getTangoPermission(EXTRA_VALUE_DATASET, REQUEST_CODE_DATASET_PERMISSION);
                updateLoadAndSaveMapButtons();
            } else if (requestCode == StartSettingsActivityRequest.STANDARD_RUN) {
                // It is ok to change the log file name at runtime.
                String logFileName = mSharedPref.getString(getString(R.string.pref_log_file_key),
                        getString(R.string.pref_log_file_default));
                mLogger.setLogFileName(logFileName);
                if (mRosStatus == RosStatus.MASTER_NOT_CONNECTED && mSnackbarRosReconnection != null) {
                    // Show snackbar with ROS reconnection button.
                    // It was dismissed when switching to the SettingsActivity.
                   mSnackbarRosReconnection.show();
                }
            }
        }

        if (requestCode == REQUEST_CODE_ADF_PERMISSION || requestCode == REQUEST_CODE_DATASET_PERMISSION) {
            if (resultCode == RESULT_CANCELED) {
                // No Tango permissions granted by the user.
                displayToastMessage(R.string.tango_permission_denied);
            }
            if (requestCode == REQUEST_CODE_ADF_PERMISSION) {
                // The user answered the ADF permission popup (the permission has not been necessarily granted).
                mAdfPermissionHasBeenAnswered = true;
            }
            if (requestCode ==  REQUEST_CODE_DATASET_PERMISSION) {
                // The user answered the dataset permission popup (the permission has not been necessarily granted).
                mDatasetPermissionHasBeenAnswered = true;
            }
            if (mAdfPermissionHasBeenAnswered && mDatasetPermissionHasBeenAnswered) {
                // Both ADF and dataset permissions popup have been answered by the user, the node
                // can start.
                Log.i(TAG, "initAndStartRosJavaNode");
                initAndStartRosJavaNode();
            }

        }
    }

    /**
     * Attempts a connection to the configured ROS Master URI, handling ROS status.
     */
    private void checkRosMasterConnection() {
        updateRosStatus(RosStatus.UNKNOWN);
        mRosConnectionLatch = new CountDownLatch(1);
        new MasterConnectionChecker(mMasterUri.toString(),
                new MasterConnectionChecker.UserHook() {
                    @Override
                    public void onSuccess(Object o) {
                        updateRosStatus(RosStatus.MASTER_CONNECTED);
                        mRosConnectionLatch.countDown();
                    }

                    @Override
                    public void onError(Throwable t) {
                        updateRosStatus(RosStatus.MASTER_NOT_CONNECTED);
                        Log.e(TAG, getString(R.string.ros_init_error));
                        displayToastMessage(R.string.ros_init_error);
                        mRosConnectionLatch.countDown();
                    }
                },
                mRosConnectionLatch
        ).runTest();
        waitForLatchUnlock(mRosConnectionLatch, "ROS CONNECTION");
    }

    private void restartTango() {
        if (mParameterNode != null) {
            try {
                mParameterNode.setPreferencesFromParameterServer();
            } catch (RuntimeException e) {
                e.printStackTrace();
            }
        }
        mTangoServiceClientNode.callTangoConnectService(TangoConnectRequest.RECONNECT);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration;
        try {
            nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
            nodeConfiguration.setMasterUri(this.nodeMainExecutorService.getMasterUri());
        } catch (RosRuntimeException e) {
            e.printStackTrace();
            Log.e(TAG, getString(R.string.network_error));
            displayToastMessage(R.string.network_error);
            return;
        }
        checkRosMasterConnection();
        if (mRosStatus == RosStatus.MASTER_NOT_CONNECTED) {
            return;
        }

        HashMap<String, String> tangoConfigurationParameters = new HashMap<String, String>();
        tangoConfigurationParameters.put(getString(R.string.pref_create_new_map_key), "boolean");
        tangoConfigurationParameters.put(getString(R.string.pref_enable_depth_key), "boolean");
        tangoConfigurationParameters.put(getString(R.string.pref_enable_color_camera_key), "boolean");
        tangoConfigurationParameters.put(getString(R.string.pref_localization_mode_key), "int_as_string");
        tangoConfigurationParameters.put(getString(R.string.pref_localization_map_uuid_key), "string");
        mParameterNode = new ParameterNode(this, tangoConfigurationParameters);
        nodeConfiguration.setNodeName(mParameterNode.getDefaultNodeName());
        nodeMainExecutor.execute(mParameterNode, nodeConfiguration);
        // ServiceClient node which is responsible for calling ros services.
        mTangoServiceClientNode = new TangoServiceClientNode();
        mTangoServiceClientNode.setCallbackListener(this);
        nodeConfiguration.setNodeName(mTangoServiceClientNode.getDefaultNodeName());
        nodeMainExecutor.execute(mTangoServiceClientNode, nodeConfiguration);
        // Create node publishing IMU data.
        mImuNode = new ImuNode(this);
        nodeConfiguration.setNodeName(mImuNode.getDefaultNodeName());
        nodeMainExecutor.execute(mImuNode, nodeConfiguration);
        // Create and start Tango ROS Node
        nodeConfiguration.setNodeName(TangoNodeletManager.NODE_NAME);
        if (TangoInitializationHelper.loadTangoSharedLibrary() !=
                TangoInitializationHelper.ARCH_ERROR &&
                TangoInitializationHelper.loadTangoRosNodeSharedLibrary()
                        != TangoInitializationHelper.ARCH_ERROR) {
            mTangoNodeletManager = new TangoNodeletManager();
            TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
            if (TangoInitializationHelper.isTangoVersionOk()) {
                nodeMainExecutor.execute(mTangoNodeletManager, nodeConfiguration, new ArrayList<NodeListener>(){{
                    add(new DefaultNodeListener() {
                        @Override
                        public void onStart(ConnectedNode connectedNode) {
                            int count = 0;
                            while (count < MAX_TANGO_CONNECTION_TRY &&
                                    !mTangoServiceClientNode.callTangoConnectService(TangoConnectRequest.CONNECT)) {
                                try {
                                    count++;
                                    Log.e(TAG, "Trying to connect to Tango, attempt " + count);
                                    Thread.sleep(200);
                                } catch (InterruptedException e) {
                                    e.printStackTrace();
                                }
                            }
                            if (count >= MAX_TANGO_CONNECTION_TRY) {
                                updateTangoStatus(TangoStatus.SERVICE_NOT_CONNECTED);
                                displayToastMessage(R.string.tango_connect_error);
                            }
                        }
                    });
                }});
            } else {
                updateTangoStatus(TangoStatus.SERVICE_NOT_CONNECTED);
                Log.e(TAG, getResources().getString(R.string.tango_version_error));
                displayToastMessage(R.string.tango_version_error);
            }
        } else {
            updateTangoStatus(TangoStatus.SERVICE_NOT_CONNECTED);
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
            getTangoPermission(EXTRA_VALUE_ADF, REQUEST_CODE_ADF_PERMISSION);
            getTangoPermission(EXTRA_VALUE_DATASET, REQUEST_CODE_DATASET_PERMISSION);
        } else {
            Intent intent = new Intent(this, SettingsActivity.class);
            startActivityForResult(intent, StartSettingsActivityRequest.FIRST_RUN);
        }
    }

    /**
     * This function initializes the tango ros node with RosJava interface.
     */
    private void initAndStartRosJavaNode() {
        this.nodeMainExecutorService.addListener(new NodeMainExecutorServiceListener() {
            @Override
            public void onShutdown(NodeMainExecutorService nodeMainExecutorService) {
                unbindFromTango();
                mLogger.saveLogToFile();
                // This ensures to kill the process started by the app.
                android.os.Process.killProcess(android.os.Process.myPid());
            }
        });
        if (mRunLocalMaster) {
            try {
                this.nodeMainExecutorService.startMaster(/*isPrivate*/ false);
                mMasterUri = this.nodeMainExecutorService.getMasterUri().toString();
                // The URI returned by getMasterUri is correct but looks 'weird',
                // e.g. 'http://android-c90553518bc67cf5:1131'.
                // Instead of showing this to the user, we show the IP address of the device,
                // which is also correct and less confusing.
                WifiManager wifiManager = (WifiManager) getApplicationContext().getSystemService(WIFI_SERVICE);
                String deviceIP = Formatter.formatIpAddress(wifiManager.getConnectionInfo().getIpAddress());
                mUriTextView = (TextView) findViewById(R.id.master_uri);
                mUriTextView.setText("http://" + deviceIP + ":11311");
            } catch (RosRuntimeException e) {
                e.printStackTrace();
                Log.e(TAG, getString(R.string.local_master_error));
                displayToastMessage(R.string.local_master_error);
                return;
            }
        }
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
     * Helper method to block the calling thread until the latch is zeroed by some other task.
     * @param latch Latch to wait for.
     * @param latchName Name to be used in log messages for the given latch.
     */
    private void waitForLatchUnlock(CountDownLatch latch, String latchName) {
        try {
            Log.i(TAG, "Waiting for " + latchName + " latch release...");
            latch.await();
            Log.i(TAG, latchName + " latch released!");
        } catch (InterruptedException ie) {
            Log.w(TAG, "Warning: continuing before " + latchName + " latch was released");
        }
    }
}
