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

import android.annotation.TargetApi;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.Bundle;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceActivity;
import android.preference.PreferenceFragment;
import android.preference.PreferenceManager;
import android.preference.SwitchPreference;
import android.support.design.widget.Snackbar;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.View;
import android.widget.Toast;

import java.util.HashMap;

/**
 * A {@link PreferenceActivity} that presents a set of application settings. On
 * handset devices, settings are presented as a single list. On tablets,
 * settings are split by category, with category headers shown to the left of
 * the list of settings.
 * <p/>
 * See <a href="http://developer.android.com/design/patterns/settings.html">
 * Android Design: Settings</a> for design guidelines and the <a
 * href="http://developer.android.com/guide/topics/ui/settings.html">Settings
 * API Guide</a> for more information on developing a Settings UI.
 */
public class SettingsActivity extends AppCompatPreferenceActivity implements
        SharedPreferences.OnSharedPreferenceChangeListener {
    private static final String TAG = SettingsActivity.class.getSimpleName();

    private SharedPreferences mSharedPref;
    private SettingsPreferenceFragment mSettingsPreferenceFragment;
    private HashMap<String, String> mUuidsNamesMap;

    /**
     * A preference value change listener that updates the preference's summary
     * to reflect its new value.
     */
    private static Preference.OnPreferenceChangeListener sBindPreferenceSummaryToValueListener = new Preference.OnPreferenceChangeListener() {
        @Override
        public boolean onPreferenceChange(Preference preference, Object value) {
            String stringValue = value.toString();

            if (preference instanceof ListPreference) {
                // For list preferences, look up the correct display value in
                // the preference's 'entries' list.
                ListPreference listPreference = (ListPreference) preference;
                int index = listPreference.findIndexOfValue(stringValue);

                // Set the summary to reflect the new value.
                preference.setSummary(
                        index >= 0
                                ? listPreference.getEntries()[index]
                                : null);
            } else {
                // For all other preferences, set the summary to the value's
                // simple string representation.
                preference.setSummary(stringValue);
            }
            return true;
        }
    };

    /**
     * Binds a preference's summary to its value. More specifically, when the
     * preference's value is changed, its summary (line of text below the
     * preference title) is updated to reflect the value. The summary is also
     * immediately updated upon calling this method. The exact display format is
     * dependent on the type of preference.
     *
     * @see #sBindPreferenceSummaryToValueListener
     */
    private static void bindPreferenceSummaryToValue(Preference preference) {
        // Set the listener to watch for value changes.
        preference.setOnPreferenceChangeListener(sBindPreferenceSummaryToValueListener);

        // Trigger the listener immediately with the preference's
        // current value.
        sBindPreferenceSummaryToValueListener.onPreferenceChange(preference,
                PreferenceManager
                        .getDefaultSharedPreferences(preference.getContext())
                        .getString(preference.getKey(), ""));
    }

    /**
     * Implements OnSharedPreferenceChangeListener to trigger a snackbar if the
     * change requires to restart the app to be applied.
     */
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, final String key) {
        if (key == getString(R.string.pref_master_is_local_key) ||
                key == getString(R.string.pref_master_uri_key) ||
                key == getString(R.string.pref_create_new_map_key) ||
                key == getString(R.string.pref_localization_mode_key) ||
                key == getString(R.string.pref_localization_map_uuid_key)) {
            boolean previouslyStarted = mSharedPref.getBoolean(getString(R.string.pref_previously_started_key), false);
            if (previouslyStarted && mSettingsPreferenceFragment.getView() != null) {
                // These changes require to restart the app.
                if (key == getString(R.string.pref_master_is_local_key) ||
                        key == getString(R.string.pref_master_uri_key)) {
                    Snackbar snackbar = Snackbar.make(mSettingsPreferenceFragment.getView(), getString(R.string.snackbar_text_restart_app), Snackbar.LENGTH_INDEFINITE);
                    View snackBarView = snackbar.getView();
                    snackBarView.setBackgroundColor(getResources().getColor(android.R.color.holo_orange_dark));
                    snackbar.show();
                }
                // These changes require to restart Tango only.
                if (key == getString(R.string.pref_create_new_map_key) ||
                        key == getString(R.string.pref_localization_mode_key) ||
                        key == getString(R.string.pref_localization_map_uuid_key)) {
                    Snackbar snackbar = Snackbar.make(mSettingsPreferenceFragment.getView(), getString(R.string.snackbar_text_restart_tango), Snackbar.LENGTH_INDEFINITE);
                    snackbar.setAction(getString(R.string.snackbar_action_text_restart_tango), new View.OnClickListener() {
                        @Override
                        public void onClick(View view) {
                            restartTango();
                        }
                    });
                    View snackBarView = snackbar.getView();
                    snackBarView.setBackgroundColor(getResources().getColor(android.R.color.holo_blue_dark));
                    snackbar.show();
                }
            }
        }
        updateMapChooserPreference();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.settings_activity);
        mSharedPref = PreferenceManager.getDefaultSharedPreferences(getBaseContext());
        mSharedPref.registerOnSharedPreferenceChangeListener(this);
        mSettingsPreferenceFragment = new SettingsPreferenceFragment();
        getFragmentManager().beginTransaction()
                .replace(R.id.fragment_container, mSettingsPreferenceFragment)
                .commit();
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
    }

    @Override
    protected void onStart() {
        super.onStart();
        boolean previouslyStarted = mSharedPref.getBoolean(getString(R.string.pref_previously_started_key), false);
        if(!previouslyStarted) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(getApplicationContext(), R.string.welcome_text_first_run, Toast.LENGTH_LONG).show();
                }
            });
            Snackbar snackbar = Snackbar.make(mSettingsPreferenceFragment.getView(), getString(R.string.snackbar_text_first_run), Snackbar.LENGTH_INDEFINITE);
            snackbar.setAction(getString(R.string.snackbar_action_text_first_run), new View.OnClickListener() {
                @Override
                public void onClick(View view) {
                    onBackPressed();
                }
            });
            snackbar.show();
        }
        Preference aboutPref = mSettingsPreferenceFragment.findPreference(getString(R.string.pref_about_app_key));
        aboutPref.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
            @Override
            public boolean onPreferenceClick(Preference preference) {
                startAboutActivity();
                return true;
            }
        });

        Intent intent = getIntent();
        mUuidsNamesMap = (HashMap<String, String>) intent.getSerializableExtra(getString(R.string.uuids_names_map));
        updateMapChooserPreference();
        mSettingsPreferenceFragment.setPreferencesSummury();
    }

    private void updateMapChooserPreference() {
        SwitchPreference createNewMapPref = (SwitchPreference) mSettingsPreferenceFragment.findPreference(getString(R.string.pref_create_new_map_key));
        if (createNewMapPref == null) return;
        boolean createNewMap = createNewMapPref.isChecked();
        ListPreference localizationModePref = (ListPreference) mSettingsPreferenceFragment.findPreference(getString(R.string.pref_localization_mode_key));
        if (localizationModePref == null) return;
        String localizationMode = localizationModePref.getValue();
        MapChooserPreference mapChooserPreference =
                (MapChooserPreference) mSettingsPreferenceFragment.findPreference(getString(R.string.pref_localization_map_uuid_key));
        if (mapChooserPreference == null) return;
        mapChooserPreference.setEnabled(!createNewMap && localizationMode.equals("3"));

        if (mUuidsNamesMap == null || mUuidsNamesMap.isEmpty()) {
            mapChooserPreference.setEnabled(false);
        } else {
            mapChooserPreference.setMapList(mUuidsNamesMap);
        }
    }

    /**
     * This method stops fragment injection in malicious applications.
     * Make sure to deny any unknown fragments here.
     */
    protected boolean isValidFragment(String fragmentName) {
        return PreferenceFragment.class.getName().equals(fragmentName)
                || SettingsPreferenceFragment.class.getName().equals(fragmentName);
    }

    /**
     * Fragment showing settings preferences.
     */
    @TargetApi(Build.VERSION_CODES.KITKAT)
    public static class SettingsPreferenceFragment extends PreferenceFragment {
        @Override
        public void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            addPreferencesFromResource(R.xml.pref_settings);
            setHasOptionsMenu(true);
        }

        // Bind the summaries of EditText/List/Dialog/Ringtone preferences
        // to their values. When their values change, their summaries are
        // updated to reflect the new value, per the Android Design
        // guidelines.
        public void setPreferencesSummury() {
            bindPreferenceSummaryToValue(findPreference(getResources().getString(R.string.pref_master_uri_key)));
            bindPreferenceSummaryToValue(findPreference(getResources().getString(R.string.pref_log_file_key)));
            bindPreferenceSummaryToValue(findPreference(getResources().getString(R.string.pref_localization_mode_key)));
            bindPreferenceSummaryToValue(findPreference(getResources().getString(R.string.pref_localization_map_uuid_key)));
        }
    }

    @Override
    public void onBackPressed() {
        boolean previouslyStarted = mSharedPref.getBoolean(getString(R.string.pref_previously_started_key), false);
        if(!previouslyStarted) {
            SharedPreferences.Editor edit = mSharedPref.edit();
            edit.putBoolean(getString(R.string.pref_previously_started_key), Boolean.TRUE);
            edit.commit();
        }
        super.onBackPressed();
    }

    public void startAboutActivity() {
        Intent intent = new Intent(this, AboutActivity.class);
        startActivity(intent);
    }

    private void restartTango() {
        Intent intent = new Intent(RunningActivity.RESTART_TANGO_ALERT);
        this.sendBroadcast(intent);
        onBackPressed();
    }
}
