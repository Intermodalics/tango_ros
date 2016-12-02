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
import android.widget.TextView;
import android.widget.Toast;

public class MainActivity extends Activity implements SetMasterUriDialog.CallbackListener,
        TryToReconnectToRosDialog.CallbackListener {
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
        // Update view.
        TextView uriTextView;
        uriTextView = (TextView) findViewById(R.id.master_uri);
        uriTextView.setText(mMasterUri);
        // Save URI preference.
        SharedPreferences sharedPref = this.getPreferences(Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPref.edit();
        editor.putString(getString(R.string.saved_uri_key), mMasterUri);
        editor.commit();
        // Start ROS and node.
        init();
        startNode();
    }

    /**
     * Implements TryToReconnectToRosDialog.CallbackListener.
     */
    @Override
    public void onTryToReconnectToRos() {
        init();
        startNode();
    }

    /**
     * Shows a dialog for setting the Master URI.
     */
    private void showSetMasterUriDialog() {
        // Get URI preference or default value if does not exist.
        SharedPreferences sharedPref = this.getPreferences(Context.MODE_PRIVATE);
        String uriValue = sharedPref.getString(getString(R.string.saved_uri_key),
                getResources().getString(R.string.saved_uri_default));
        Bundle bundle = new Bundle();
        bundle.putString(getString(R.string.saved_uri_key), uriValue);
        FragmentManager manager = getFragmentManager();
        SetMasterUriDialog setMasterUriDialog = new SetMasterUriDialog();
        setMasterUriDialog.setArguments(bundle);
        setMasterUriDialog.show(manager, "MatserUriDialog");
    }

    /**
     * Shows a dialog for trying to reconnect to ros master.
     */
    private void showTryToReconnectToRosDialog() {
        SharedPreferences sharedPref = this.getPreferences(Context.MODE_PRIVATE);
        String uriValue = sharedPref.getString(getString(R.string.saved_uri_key),
                getResources().getString(R.string.saved_uri_default));
        Bundle bundle = new Bundle();
        bundle.putString(getString(R.string.saved_uri_key), uriValue);
        FragmentManager manager = getFragmentManager();
        TryToReconnectToRosDialog setTryToReconnectToRosDialog = new TryToReconnectToRosDialog();
        setTryToReconnectToRosDialog.setArguments(bundle);
        setTryToReconnectToRosDialog.show(manager, "TryToReconnectToRosDialog");
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
        // Update publisher configuration according to current preferences.
        PrefsFragment prefsFragment = (PrefsFragment) getFragmentManager().findFragmentById(R.id.preferencesFrame);
        mPublishConfig = prefsFragment.getPublisherConfigurationFromPreferences();
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
                showTryToReconnectToRosDialog();
            }
        } else {
            Log.e(TAG, "Master URI is null");
        }
    }

    public void startNode() {
        if (mIsNodeInitialised) {
            TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
            mJniInterface.startPublishing();
        }
    }

    public void applySettings() {
        mJniInterface.updatePublisherConfiguration(mPublishConfig);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_activity);
        mPublishConfig = new PublisherConfiguration();
        getFragmentManager().beginTransaction().replace(R.id.preferencesFrame, new PrefsFragment()).commit();
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
        startNode();
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mIsNodeInitialised) {
            mJniInterface.stopPublishing();
            mJniInterface.tangoDisconnect();
            unbindService(mTangoServiceConnection);
        }
    }
}