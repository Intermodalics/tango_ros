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
import android.app.DialogFragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;


/**
 * Displays custom view with progress bar and text information while saving an adf.
 */
public class SaveMapDialog extends DialogFragment implements View.OnClickListener {
    private static final String TAG = SaveMapDialog.class.getSimpleName();

    EditText mNameEditText;
    CallbackListener mCallbackListener;

    public interface CallbackListener {
        void onMapNameOk(String name);
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        mCallbackListener = (CallbackListener) activity;
    }

    @Override
    public View onCreateView(LayoutInflater inflator, ViewGroup container,
                             Bundle savedInstanceState) {
        View dialogView = inflator.inflate(R.layout.dialog_save_map, null);
        getDialog().setTitle("Enter the name of the map");
        mNameEditText = (EditText) dialogView.findViewById(R.id.map_name);
        dialogView.findViewById(R.id.Ok).setOnClickListener(this);
        setCancelable(false);
        return dialogView;
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.Ok:
                mCallbackListener.onMapNameOk(
                        mNameEditText.getText().toString());
                dismiss();
                break;
        }
    }
}