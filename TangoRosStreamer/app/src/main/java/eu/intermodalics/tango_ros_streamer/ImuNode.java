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

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.SystemClock;

import org.apache.commons.logging.Log;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import sensor_msgs.Imu;

/**
 *
 */
public class ImuNode  extends AbstractNodeMain implements NodeMain, SensorEventListener {
    private static final String NODE_NAME = "android";

    private ConnectedNode mConnectedNode;
    private Publisher<Imu> mImuPublisher;
    private Imu mImuMessage;
    private Log mLog;

    private SensorManager mSensorManager;
    private Sensor mRotationSensor;
    private Sensor mGyroscopeSensor;
    private Sensor mAccelerometerSensor;
    private boolean mNewRotationData = false;
    private boolean mNewGyroscopeData = false;
    private boolean mNewAccelerometerData = false;

    public ImuNode(Activity activity) {
        mSensorManager = (SensorManager) activity.getSystemService(Context.SENSOR_SERVICE);

        mRotationSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        mGyroscopeSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mAccelerometerSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        mSensorManager.registerListener(this, mRotationSensor, SensorManager.SENSOR_DELAY_FASTEST);
        mSensorManager.registerListener(this, mGyroscopeSensor, SensorManager.SENSOR_DELAY_FASTEST);
        mSensorManager.registerListener(this, mAccelerometerSensor, SensorManager.SENSOR_DELAY_FASTEST);
    }

    public GraphName getDefaultNodeName() { return GraphName.of(NODE_NAME); }

    public void onStart(ConnectedNode connectedNode) {
        mConnectedNode = connectedNode;
        mLog = connectedNode.getLog();
        mImuPublisher = connectedNode.newPublisher("android/imu", Imu._TYPE);
        mImuMessage = mConnectedNode.getTopicMessageFactory().newFromType(Imu._TYPE);
    }

    @Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Not used.
	}

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ROTATION_VECTOR:
                mNewRotationData = true;
                float[] quaternion = new float[4];
                SensorManager.getQuaternionFromVector(quaternion, event.values);
                mImuMessage.getOrientation().setW(quaternion[0]);
                mImuMessage.getOrientation().setX(quaternion[1]);
                mImuMessage.getOrientation().setY(quaternion[2]);
                mImuMessage.getOrientation().setZ(quaternion[3]);
                mImuMessage.setOrientationCovariance(
                        new double[]{0.001, 0., 0.,
                                     0., 0.001, 0.,
                                     0., 0., 0.001}
                );
                break;
            case Sensor.TYPE_GYROSCOPE:
                mNewGyroscopeData = true;
                mImuMessage.getAngularVelocity().setX(event.values[0]);
                mImuMessage.getAngularVelocity().setY(event.values[1]);
                mImuMessage.getAngularVelocity().setZ(event.values[2]);
                mImuMessage.setAngularVelocityCovariance(
                        new double[]{0.0025, 0., 0.,
                                     0., 0.0025, 0.,
                                     0., 0., 0.0025}
                );
                break;
            case Sensor.TYPE_ACCELEROMETER:
                mNewAccelerometerData = true;
                mImuMessage.getLinearAcceleration().setX(event.values[0]);
                mImuMessage.getLinearAcceleration().setY(event.values[1]);
                mImuMessage.getLinearAcceleration().setZ(event.values[2]);
                mImuMessage.setLinearAccelerationCovariance(
                        new double[]{0.01, 0., 0.,
                                     0., 0.01, 0.,
                                     0., 0., 0.01});
                break;
            default:
                break;
        }
        if (mNewRotationData && mNewGyroscopeData && mNewAccelerometerData) {
            long timeOffset = System.currentTimeMillis() - SystemClock.uptimeMillis();
            mImuMessage.getHeader().setStamp(Time.fromMillis(timeOffset + event.timestamp / 1000000));
            mImuMessage.getHeader().setFrameId("imu");
            mImuPublisher.publish(mImuMessage);
            mNewRotationData = false;
            mNewGyroscopeData = false;
            mNewAccelerometerData = false;
        }
    }
}
