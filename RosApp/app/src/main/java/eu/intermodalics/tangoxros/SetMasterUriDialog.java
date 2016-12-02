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
import android.widget.EditText;

/**
 * Queries the user for a Master URI.
 */
public class SetMasterUriDialog extends DialogFragment implements OnClickListener {

    EditText mUriEditText;
    CallbackListener mCallbackListener;

    interface CallbackListener {
        public void onMasterUriConnect(String uri);
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        mCallbackListener = (CallbackListener) activity;
    }

    @Override
    public View onCreateView(LayoutInflater inflator, ViewGroup container,
                             Bundle savedInstanceState) {
        View dialogView = inflator.inflate(R.layout.set_master_uri_dialog, null);
        getDialog().setTitle(R.string.set_master_uri_dialogTitle);
        mUriEditText = (EditText) dialogView.findViewById(R.id.uri);
        dialogView.findViewById(R.id.connect).setOnClickListener(this);
        setCancelable(false);
        String uri = this.getArguments().getString(getString(R.string.saved_uri_key));
        if (uri != null) {
            mUriEditText.setText(uri);
        }
        return dialogView;
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.connect:
                mCallbackListener.onMasterUriConnect(
                        mUriEditText.getText().toString());
                dismiss();
                break;
        }
    }
}