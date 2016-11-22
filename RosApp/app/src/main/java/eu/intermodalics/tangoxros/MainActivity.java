package eu.intermodalics.tangoxros;

import android.app.Activity;
import android.app.FragmentManager;
import android.content.ComponentName;
import android.content.Context;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.IBinder;
import android.text.format.Formatter;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.Toast;

public class MainActivity extends Activity implements SetMasterUriDialog.CallbackListener {
    private static final String TAG = MainActivity.class.getSimpleName();
    private static final String MASTER_URI_PREFIX = "__master:=";
    private static final String IP_PREFIX = "__ip:=";

    private JNIInterface mJniInterface;
    private String mMasterUri;
    private boolean mIsNodeInitialised = false;
    private PublisherConfiguration mPublishConfig;

    /**
     * Implements SetMasterUriDialog.CallbackListener.
     */
    @Override
    public void onMasterUriConnect(String uri) {
        mMasterUri = uri;
        // Save URI preference.
        SharedPreferences sharedPref = this.getPreferences(Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPref.edit();
        editor.putString(getString(R.string.saved_uri), mMasterUri);
        editor.commit();
        // Start ROS and node.
        init();
        onResume();
    }

    /**
     * Shows a dialog for setting the Master URI.
     */
    private void showSetMasterUriDialog() {
        // Get URI preference or default value if does not exist.
        SharedPreferences sharedPref = this.getPreferences(Context.MODE_PRIVATE);
        String defaultUriValue = getResources().getString(R.string.saved_uri_default);
        String uriValue = sharedPref.getString(getString(R.string.saved_uri), defaultUriValue);

        Bundle bundle = new Bundle();
        bundle.putString(getString(R.string.saved_uri), uriValue);
        FragmentManager manager = getFragmentManager();
        SetMasterUriDialog setMasterUriDialog = new SetMasterUriDialog();
        setMasterUriDialog.setArguments(bundle);
        setMasterUriDialog.show(manager, "MatserUriDialog");
    }

    /**
     * Tango Service connection.
     */
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            // Synchronization around MainActivity object is to avoid
            // Tango disconnect in the middle of the connecting operation.
            if(!mJniInterface.onTangoServiceConnected(service)) {
                Log.e(TAG, getResources().getString(R.string.tango_service_error));
                Toast.makeText(getApplicationContext(), R.string.tango_service_error, Toast.LENGTH_SHORT).show();
                onDestroy();
            }
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };

    public boolean initNode() {
        if(!mJniInterface.initNode(this, mPublishConfig)) {
            Log.e(TAG, getResources().getString(R.string.tango_node_error));
            Toast.makeText(getApplicationContext(), R.string.tango_node_error, Toast.LENGTH_SHORT).show();
            return false;
        }
        return true;
    }

    public void init() {
        if (mMasterUri != null) {
            WifiManager wm = (WifiManager) getSystemService(WIFI_SERVICE);
            String ip_address = Formatter.formatIpAddress(wm.getConnectionInfo().getIpAddress());
            if (mJniInterface.initRos(MASTER_URI_PREFIX + mMasterUri, IP_PREFIX + ip_address)) {
                mIsNodeInitialised = initNode();
            } else {
                Log.e(TAG, getResources().getString(R.string.tango_ros_error));
                Toast.makeText(getApplicationContext(), R.string.tango_ros_error, Toast.LENGTH_SHORT).show();
            }
        } else {
            Log.e(TAG, "Master URI is null");
        }
    }

    public void applySettings() {
        onPause();
        mIsNodeInitialised = initNode();
        if (mIsNodeInitialised) {
            onResume();
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_activity);
        mPublishConfig = new PublisherConfiguration();
        // Set callback for device pose switch.
        Switch switchDevicePose = (Switch) findViewById(R.id.switch_device_pose);
        switchDevicePose.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    mPublishConfig.publishDevicePose = true;
                    Log.i(TAG, "Publish device pose is switched on");
                } else {
                    mPublishConfig.publishDevicePose = false;
                    Log.i(TAG, "Publish device pose is switched off");
                }
            }
        });
        // Set callback for point cloud switch.
        Switch switchPointCloud = (Switch) findViewById(R.id.switch_pointcloud);
        switchPointCloud.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    mPublishConfig.publishPointCloud = true;
                    Log.i(TAG, "Publish point cloud is switched on");
                } else {
                    mPublishConfig.publishPointCloud = false;
                    Log.i(TAG, "Publish point cloud is switched off");
                }
            }
        });
        // Set callback for fisheye camera switch.
        Switch switchFisheyeCamera = (Switch) findViewById(R.id.switch_fisheye_camera);
        switchFisheyeCamera.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    mPublishConfig.publishCamera |= PublisherConfiguration.CAMERA_FISHEYE;
                    Log.i(TAG, "Publish fisheye camera is switched on");
                } else {
                    mPublishConfig.publishCamera &= ~PublisherConfiguration.CAMERA_FISHEYE;
                    Log.i(TAG, "Publish fisheye camera is switched off");
                }
            }
        });

        // Set callback for color camera switch.
        Switch switchColorCamera = (Switch) findViewById(R.id.switch_color_camera);
        switchColorCamera.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    mPublishConfig.publishCamera |= PublisherConfiguration.CAMERA_COLOR;
                    Log.i(TAG, "Publish color camera is switched on");
                } else {
                    mPublishConfig.publishCamera &= ~PublisherConfiguration.CAMERA_COLOR;
                    Log.i(TAG, "Publish color camera is switched off");
                }
            }
        });
        // Set callback for apply button.
        Button buttonApply = (Button)findViewById(R.id.apply);
        buttonApply.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v){
                applySettings();
            }
        });
        // Request master URI from user.
        showSetMasterUriDialog();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mIsNodeInitialised) {
            TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
            new Thread(new Runnable() {
                @Override
                public void run() {
                    while (mJniInterface.isRosOk()) {
                        mJniInterface.publish();
                    }
                }
            }).start();
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mIsNodeInitialised) {
            mJniInterface.tangoDisconnect();
            unbindService(mTangoServiceConnection);
        }
    }
}