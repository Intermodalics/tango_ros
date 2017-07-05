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

package eu.intermodalics.tango_ros_streamer.android;

import android.app.DialogFragment;
import android.content.Context;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import eu.intermodalics.tango_ros_streamer.R;


/**
 * Displays dialog requesting name of the map before saving.
 */
public class SaveMapDialog extends DialogFragment implements View.OnClickListener {
    private static final String TAG = SaveMapDialog.class.getSimpleName();

    Button mOkButton;
    EditText mNameEditText;
    CallbackListener mCallbackListener;

    public interface CallbackListener {
        void onClickOkSaveMapDialog(String name);
    }

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        mCallbackListener = (CallbackListener) context;
    }

    @Override
    public View onCreateView(LayoutInflater inflator, ViewGroup container,
                             Bundle savedInstanceState) {
        View dialogView = inflator.inflate(R.layout.dialog_save_map, null);
        getDialog().setTitle(R.string.save_map_dialog_title);
        mOkButton = (Button) dialogView.findViewById(R.id.save_map_ok);
        mOkButton.setOnClickListener(this);
        dialogView.findViewById(R.id.save_map_cancel).setOnClickListener(this);
        mNameEditText = (EditText) dialogView.findViewById(R.id.map_name);
        DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
        mNameEditText.setText(dateFormat.format(new Date()) + "_map");
        mNameEditText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence charSequence, int i, int i1, int i2) {
                // Not used.
            }

            @Override
            public void onTextChanged(CharSequence charSequence, int i, int i1, int i2) {
                // Not used.
            }

            @Override
            public void afterTextChanged(Editable editable) {
                if (mNameEditText.getText().toString().isEmpty()) {
                    mOkButton.setEnabled(false);
                } else {
                    mOkButton.setEnabled(true);
                }
            }
        });
        return dialogView;
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.save_map_ok:
                Log.i(TAG, "OK");
                mCallbackListener.onClickOkSaveMapDialog(
                        mNameEditText.getText().toString());
                dismiss();
                break;
            case R.id.save_map_cancel:
                Log.i(TAG, "CANCEL");
                dismiss();
                break;
        }
    }
}