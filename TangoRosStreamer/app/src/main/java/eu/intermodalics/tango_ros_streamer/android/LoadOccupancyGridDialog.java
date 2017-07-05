/*
 * Copyright 2017 Intermodalics All Rights Reserved.
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
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;

import eu.intermodalics.tango_ros_streamer.R;


/**
 * Displays dialog requesting name of the map before saving.
 */
public class LoadOccupancyGridDialog extends DialogFragment implements View.OnClickListener {
    private static final String TAG = LoadOccupancyGridDialog.class.getSimpleName();
    CallbackListener mCallbackListener;
    Context mContext;

    public interface CallbackListener {
        void onClickItemLoadOccupancyGridDialog(String name);
    }

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        mContext = context;
        mCallbackListener = (CallbackListener) context;
    }

    @Override
    public View onCreateView(LayoutInflater inflator, ViewGroup container,
                             Bundle savedInstanceState) {
        View dialogView = inflator.inflate(R.layout.dialog_load_occupancy_grid, null);
        getDialog().setTitle(R.string.load_occupancy_grid_dialog_title);

        Button cancelButton = (Button) dialogView.findViewById(R.id.load_occupancy_grid_cancel);
        cancelButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                dismiss();
            }
        });

        Bundle bundle = getArguments();
        if(bundle.getBoolean(getString(R.string.show_load_occupancy_grid_empty_key))) {
            TextView errorTextView = (TextView) dialogView.findViewById(R.id.load_occupancy_grid_empty_message);
            errorTextView.setVisibility(View.VISIBLE);
        }

        if(bundle.getBoolean(getString(R.string.show_load_occupancy_grid_error_key))) {
            TextView errorTextView = (TextView) dialogView.findViewById(R.id.load_occupancy_grid_error_message);
            errorTextView.setVisibility(View.VISIBLE);
        }

        java.util.ArrayList<java.lang.String> nameList = bundle.getStringArrayList(getString(R.string.list_names_occupancy_grid_key));
        ListView listView = (ListView) dialogView.findViewById(R.id.load_occupancy_grid_list);
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(mContext, R.layout.custom_list_item, nameList);
        listView.setAdapter(adapter);
        listView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {
                mCallbackListener.onClickItemLoadOccupancyGridDialog(
                        adapterView.getItemAtPosition(i).toString());
                dismiss();
            }
        });
        return dialogView;
    }

    @Override
    public void onClick(View v) {

    }
}