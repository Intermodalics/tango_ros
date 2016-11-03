package com.intermodalics.tangoxros;

import android.app.Activity;
import android.content.ComponentName;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;

public class MainActivity extends Activity {
    private static final String TAG = MainActivity.class.getSimpleName();
    private JNIInterface jniInterface;

    // Tango Service connection.
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            // Synchronization around MeshBuilderActivity object is to avoid
            // Tango disconnect in the middle of the connecting operation.
            jniInterface.onTangoServiceConnected(service);
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //setContentView(R.layout.activity_tango_x_ros);
        jniInterface.initRos("__master:=http://im-desktop-005:11311", "__ip:=192.168.168.185");
        jniInterface.onCreate(this);
    }

    @Override
    protected void onResume() {
        super.onResume();
        TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (jniInterface.isRosOk()) {
                    jniInterface.publish();
                }
            }
        }).start();
    }

    @Override
    protected void onPause() {
        super.onPause();
        jniInterface.tangoDisconnect();
        unbindService(mTangoServiceConnection);
    }
}