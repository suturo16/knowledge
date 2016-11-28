/*
 * Copyright (C) 2014 simba.
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

package com.github.rosjava.object_state.perception_subscriber;

import org.apache.commons.logging.Log;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Pose;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import perception_msgs.ObjectDetection;

import org.knowrob.prolog.PrologInterface;
import org.ros.concurrent.CancellableLoop;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class Listener extends AbstractNodeMain {
	
	ConnectedNode node;
	BlockingQueue<ObjectDetection> detections;
	Thread updateKnowRobObjDetections;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rosjava/listener");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		// save reference to the ROS node
		this.node = connectedNode;
		
		this.detections = new LinkedBlockingQueue<ObjectDetection>();
		
		
		// wait for node to be ready
		try {
			while(node ==null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		  
	    final Log log = connectedNode.getLog();
	    Subscriber<ObjectDetection> subscriber = connectedNode.newSubscriber("dummy_object_detections", ObjectDetection._TYPE);
	    subscriber.addMessageListener(new MessageListener<ObjectDetection>() {
	    	@Override
	    	public void onNewMessage(perception_msgs.ObjectDetection message) {
	    		try {
	    			detections.put(message);
	    			log.info("Logged \""+ message.getName() +"\"");
	    			log.info("Detections now has "+ detections.size() +".");
	    		} catch (InterruptedException e) {
	    			e.printStackTrace();
	    		}
	    		log.info("I saw: \"" + message.getName() + "\"");
	    	}
	    });
    
	}
}
