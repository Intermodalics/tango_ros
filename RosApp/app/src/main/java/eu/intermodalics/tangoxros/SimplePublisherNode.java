package eu.intermodalics.tangoxros;

/*
 * Copyright (C) 2014 Oliver Degener.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import java.text.SimpleDateFormat;
import java.util.Date;

import std_msgs.String;

public class SimplePublisherNode extends AbstractNodeMain implements NodeMain {

    private static final java.lang.String TAG = SimplePublisherNode.class.getSimpleName();

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("SimplePublisher/TimeLoopNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Publisher<String> publisher = connectedNode.newPublisher(GraphName.of("time"), String._TYPE);

        final CancellableLoop loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                // retrieve current system time
                java.lang.String time = new SimpleDateFormat("HH:mm:ss").format(new Date());

                Log.i(TAG, "publishing the current time: " + time);

                // create and publish a simple string message
                String str = publisher.newMessage();
                str.setData("The current time is: " + time);
                publisher.publish(str);

                // go to sleep for one second
                Thread.sleep(1000);
            }
        };
        connectedNode.executeCancellableLoop(loop);
    }

}