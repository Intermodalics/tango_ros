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

import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.Preference;
import android.preference.PreferenceFragment;
import android.preference.PreferenceManager;
import android.preference.SwitchPreference;
import android.util.Log;

/**
 * Created by intermodalics on 12/1/16.
 */
public class PrefsFragment extends PreferenceFragment implements SharedPreferences.OnSharedPreferenceChangeListener {
    private static final String TAG = PrefsFragment.class.getSimpleName();

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // Load the preferences from an XML resource
        addPreferencesFromResource(R.xml.preferences);
        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(getActivity());
        pref.registerOnSharedPreferenceChangeListener(this);
    }

    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences,
                                          String key) {
        Preference pref = findPreference(key);
        if (pref instanceof SwitchPreference) {
            SwitchPreference swPref = (SwitchPreference) pref;
            swPref.setChecked(sharedPreferences.getBoolean(key, false));
        }
    }

    public PublisherConfiguration getPublisherConfigurationFromPreferences() {
        SharedPreferences sharedPref = getPreferenceManager().getDefaultSharedPreferences(getActivity());
        PublisherConfiguration publisherConfiguration = new PublisherConfiguration();
        publisherConfiguration.publishDevicePose = sharedPref.getBoolean(getString(R.string.publish_device_pose_key), false);
        publisherConfiguration.publishPointCloud = sharedPref.getBoolean(getString(R.string.publish_pointcloud_key), false);

        if(sharedPref.getBoolean(getString(R.string.publish_fisheye_camera_key), false)) {
            publisherConfiguration.publishCamera |= PublisherConfiguration.CAMERA_FISHEYE;
        } else {
            publisherConfiguration.publishCamera &= ~PublisherConfiguration.CAMERA_FISHEYE;
        }
        if(sharedPref.getBoolean(getString(R.string.publish_color_camera_key), false)) {
            publisherConfiguration.publishCamera |= PublisherConfiguration.CAMERA_COLOR;
        } else {
            publisherConfiguration.publishCamera &= ~PublisherConfiguration.CAMERA_COLOR;
        }
        Log.i(TAG, publisherConfiguration.toString());
        return publisherConfiguration;
    }

    public void setPreferencesFromPublsherConfiguration(PublisherConfiguration publishConfig) {
        SharedPreferences sharedPref = getPreferenceManager().getDefaultSharedPreferences(getActivity());
        SharedPreferences.Editor editor = sharedPref.edit();

        editor.putBoolean(getString(R.string.publish_device_pose_key), publishConfig.publishDevicePose);
        editor.putBoolean(getString(R.string.publish_pointcloud_key), publishConfig.publishPointCloud);
        editor.putBoolean(getString(R.string.publish_fisheye_camera_key), (publishConfig.publishCamera & PublisherConfiguration.CAMERA_FISHEYE) != 0);
        editor.putBoolean(getString(R.string.publish_color_camera_key), (publishConfig.publishCamera & PublisherConfiguration.CAMERA_COLOR) != 0);

        editor.commit();
    }

}