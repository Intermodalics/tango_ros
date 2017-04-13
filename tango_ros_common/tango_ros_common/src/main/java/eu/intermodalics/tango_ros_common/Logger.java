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

package eu.intermodalics.tango_ros_common;

import android.app.Activity;
import android.os.Environment;
import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This class is responsible for retrieving the logcat text and writing it to file.
 */
public class Logger {
    private static final String TAG = Logger.class.getSimpleName();
    private static final long LOG_THREAD_DURATION = 15000; // ms
    private static final int LOG_THREAD_SLEEP_DURATION = 500; // ms

    private Activity mActivity;
    private TextView mLogTextView;
    private String mLogCommand;
    private String mLogFileName;
    private int mLogTextMaxLength;
    private StringBuilder mLogStringBuilder;
    private File mLogFile;
    private Thread mLogThread;

    public Logger(Activity activity, TextView logTextView, String tagsToLog,
                  String logFileName, int logTextMaxLength) {
        mActivity = activity;
        mLogTextView = logTextView;
        mLogCommand = "logcat -d -s " + tagsToLog;
        mLogFileName = logFileName;
        mLogTextMaxLength = logTextMaxLength;
        mLogStringBuilder = new StringBuilder();
        mLogThread = new Thread(new Runnable() {
            @Override
            public void run() {
                long endTime = System.currentTimeMillis() + (LOG_THREAD_DURATION);
                while (System.currentTimeMillis() <= endTime) {
                    updateLogText();
                    mActivity.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mLogTextView.setText(getLogText());
                        }
                    });
                    try {
                        mLogThread.sleep(LOG_THREAD_SLEEP_DURATION);
                    } catch (InterruptedException e) {
                        Log.e(TAG, e.toString());
                    }
                }
            }
        });
    }

    public void start() {
        mLogThread.start();
    }

    public void updateLogText() {
        try {
            Process process = Runtime.getRuntime().exec(mLogCommand);
            BufferedReader bufferedReader = new BufferedReader(
                    new InputStreamReader(process.getInputStream()));
            String line = "";
            while ((line = bufferedReader.readLine()) != null) {
                mLogStringBuilder.append(line + "\n");
            }
            // The following allows to keep only the end of the logcat text.
            mLogStringBuilder.reverse();
            mLogStringBuilder.setLength(mLogTextMaxLength);
            mLogStringBuilder.reverse();
        } catch (IOException e) {
            Log.e(TAG, e.toString());
        }
    }

    public void setLogFileName(String logFileName) {
        mLogFileName = logFileName;
    }

    public String getLogText() {
        return mLogStringBuilder.toString();
    }

    public void saveLogToFile() {
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd-hh-mm-ss");
        mLogFile = new File(Environment.getExternalStorageDirectory().getPath(), mLogFileName
                + "_" + dateFormat.format(new Date()) + ".txt");
        if (!mLogFile.exists()) {
            try {
                mLogFile.createNewFile();
            } catch (IOException e) {
                Log.e(TAG, e.toString());
            }
        }
        try {
            BufferedWriter buf = new BufferedWriter(new FileWriter(mLogFile, false));
            buf.write(getLogText());
            Log.i(TAG, "Saved log to file: " + mLogFile.getAbsolutePath());
            buf.close();
        } catch (IOException e) {
            Log.e(TAG, e.toString());
        }
    }

    public File getLogFile() {
        if (mLogFile == null) {
            Log.e(TAG, "Log file is null");
            mActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(mActivity.getApplicationContext(), "Unexpected error : log file is null", Toast.LENGTH_SHORT).show();
                }
            });
            return null;
        }
        return mLogFile;
    }
}