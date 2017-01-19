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

import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.text.method.LinkMovementMethod;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.TextView;

public class AboutActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.about_activity);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        // Make links in text views clickable.
        TextView textViewRos = (TextView) findViewById(R.id.text_ros_about);
        textViewRos.setMovementMethod(LinkMovementMethod.getInstance());
        TextView textViewTango = (TextView) findViewById(R.id.text_tango_about);
        textViewTango.setMovementMethod(LinkMovementMethod.getInstance());
        TextView textViewRosjava = (TextView) findViewById(R.id.text_rosjava_about);
        textViewRosjava.setMovementMethod(LinkMovementMethod.getInstance());

        Button onlineDocButton = (Button) findViewById(R.id.online_doc_about);
        onlineDocButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //TODO: Add address to ros wiki.
                //Uri uri = Uri.parse(getString(R.string.app_wiki_address));
                //Intent intent = new Intent(Intent.ACTION_VIEW, uri);
                //startActivity(intent);
            }
        });

        Button rateAppButton = (Button) findViewById(R.id.rate_app_about);
        rateAppButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                startRateAppActivity();
            }
        });

        ImageButton intermodalicsWebsiteButton = (ImageButton) findViewById(R.id.logo_intermodalics_about);
        intermodalicsWebsiteButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Uri uri = Uri.parse(getString(R.string.intermodalics_website_address));
                Intent intent = new Intent(Intent.ACTION_VIEW, uri);
                startActivity(intent);
            }
        });

        ImageButton ekumenWebsiteButton = (ImageButton) findViewById(R.id.logo_ekumen_about);
        ekumenWebsiteButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Uri uri = Uri.parse(getString(R.string.ekumen_website_address));
                Intent intent = new Intent(Intent.ACTION_VIEW, uri);
                startActivity(intent);
            }
        });
    }

    private void startRateAppActivity() {
        Intent intent = new Intent(Intent.ACTION_VIEW, Uri.parse("market://details?id=" + this.getPackageName()));
        this.startActivity(intent);
    }
}
