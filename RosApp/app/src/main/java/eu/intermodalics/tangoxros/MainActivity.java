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

import android.app.FragmentManager;
import android.content.ComponentName;
import android.content.Context;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

public class MainActivity extends RosActivity implements SetMasterUriDialog.CallbackListener,
        TryToReconnectToRosDialog.CallbackListener {
    private static final String TAG = MainActivity.class.getSimpleName();

    PrefsFragment mPrefsFragment = new PrefsFragment();

    TangoRosNode mTangoRosNode;
    private String mMasterUri = "";
    private PublisherConfiguration mPublishConfig = new PublisherConfiguration();
    boolean mIsTangoServiceBound = false;

    public MainActivity() {
        super("TangoxRos", "TangoxRos");
    }

    protected MainActivity(String notificationTicker, String notificationTitle) {
        super(notificationTicker, notificationTitle);
    }

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
        initAndStartRosJavaNode();
    }

    /**
     * Implements TryToReconnectToRosDialog.CallbackListener.
     */
    @Override
    public void onTryToReconnectToRos() {
        initAndStartRosJavaNode();
    }

    /**
     * Shows a dialog to request master URI from user.
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
        setMasterUriDialog.show(manager, "MasterUriDialog");
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
            if(!mTangoRosNode.setBinderTangoService(service)) {
                Log.e(TAG, getResources().getString(R.string.tango_service_error));
                Toast.makeText(getApplicationContext(), R.string.tango_service_error, Toast.LENGTH_SHORT).show();
                onDestroy();
            } else {
                mIsTangoServiceBound = true;
            }
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_activity);
        getFragmentManager().beginTransaction().replace(R.id.preferencesFrame, new PrefsFragment()).commit();
        // Set callback for apply button.
        Button buttonApply = (Button)findViewById(R.id.apply);
        buttonApply.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                applySettings();
            }
        });
    }

    @Override
    protected void onStart() {
        super.onStart();
        mPrefsFragment = (PrefsFragment) getFragmentManager().findFragmentById(R.id.preferencesFrame);
        if (mMasterUri.isEmpty()) {
            showSetMasterUriDialog();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (mIsTangoServiceBound) {
            unbindService(mTangoServiceConnection);
        }
    }

    public void applySettings() {
        mPublishConfig = mPrefsFragment.getPublisherConfigurationFromPreferences();
        mTangoRosNode.updatePublisherConfiguration(mPublishConfig);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(this.nodeMainExecutorService.getMasterUri());
        nodeConfiguration.setNodeName(TangoRosNode.NODE_NAME);
        mTangoRosNode = new TangoRosNode();
        TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
        nodeMainExecutor.execute(mTangoRosNode, nodeConfiguration);
    }

    // This function allows initialization of the node with RosJava interface without using MasterChooser,
    // and is compatible with current Master Uri setter interface.
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
                    MainActivity.this.init(nodeMainExecutorService);
                    return null;
                }
            }.execute();

        } else {
            Log.e(TAG, "Master URI is null");
        }
    }

    @Override
    public void startMasterChooser() {
        // onMasterUriConnect already connects to master; overriding this method with an empty one prevents MasterChooser from running.
    }
}