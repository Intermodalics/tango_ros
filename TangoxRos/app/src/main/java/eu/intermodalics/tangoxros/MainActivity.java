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

public class MainActivity extends Activity implements SetMasterUriDialog.CallbackListener {
    private static final String TAG = MainActivity.class.getSimpleName();
    private static final String MASTER_URI_PREFIX = "__master:=";
    private static final String IP_PREFIX = "__ip:=";

    private JNIInterface mJniInterface;
    private String mMasterUri;
    private boolean isInitialised = false;

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
            mJniInterface.onTangoServiceConnected(service);
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };

    public void init() {
        if (mMasterUri != null) {
            WifiManager wm = (WifiManager) getSystemService(WIFI_SERVICE);
            String ip_address = Formatter.formatIpAddress(wm.getConnectionInfo().getIpAddress());
            if (mJniInterface.initRos(MASTER_URI_PREFIX + mMasterUri, IP_PREFIX + ip_address)) {
                mJniInterface.initNode(this);
                isInitialised = true;
            } else {
                Log.e(TAG, "Unable to init ROS!");
            }
        } else {
            Log.e(TAG, "Master URI is null");
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        showSetMasterUriDialog();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (isInitialised) {
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
        if (isInitialised) {
            mJniInterface.tangoDisconnect();
            unbindService(mTangoServiceConnection);
        }
    }
}