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
import android.app.DialogFragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.widget.TextView;

/**
 * Ask the user to retry connecting to ros master.
 */
public class TryReconnectingToRosDialog extends DialogFragment implements OnClickListener {

    TextView mUriTextView;
    CallbackListener mCallbackListener;

    interface CallbackListener {
        public void onTryReconnectingToRos();
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        mCallbackListener = (CallbackListener) activity;
    }

    @Override
    public View onCreateView(LayoutInflater inflator, ViewGroup container,
                             Bundle savedInstanceState) {
        View dialogView = inflator.inflate(R.layout.try_reconnecting_ros_dialog, null);
        getDialog().setTitle(R.string.try_reconnecting_ros_dialogTitle);
        mUriTextView = (TextView) dialogView.findViewById(R.id.master_uri);
        mUriTextView.setText(this.getArguments().getString(getString(R.string.saved_uri)));
        dialogView.findViewById(R.id.try_reconnect).setOnClickListener(this);
        setCancelable(false);
        return dialogView;
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.try_reconnect:
                mCallbackListener.onTryReconnectingToRos();
                dismiss();
                break;
        }
    }
}