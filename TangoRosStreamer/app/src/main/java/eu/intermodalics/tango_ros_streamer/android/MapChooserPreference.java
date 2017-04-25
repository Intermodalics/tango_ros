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

import android.content.Context;
import android.preference.ListPreference;
import android.util.AttributeSet;

import java.util.ArrayList;
import java.util.Map;


public class MapChooserPreference extends ListPreference {
    private static final String TAG = MapChooserPreference.class.getSimpleName();

    public MapChooserPreference (Context context, AttributeSet attrs) {
        super(context, attrs);
        setValueIndex(0);
    }

    public MapChooserPreference (Context context) {
        this(context, null);
    }

    public void setMapList(Map<String, String> uuidNameMap) {
        if (uuidNameMap != null) {
            ArrayList<String> uuids = new ArrayList<String>();
            ArrayList<String> names = new ArrayList<String>();
            for (String uuid : uuidNameMap.keySet()) {
                uuids.add(uuid);
                names.add(uuidNameMap.get(uuid));
            }
            setEntries(names.toArray(new CharSequence[names.size()]));
            setEntryValues(uuids.toArray(new CharSequence[uuids.size()]));
        }
    }
}
