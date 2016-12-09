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
import java.sql.Time;
import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import suturo_perception_msgs.ObjectDetection;

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
	    	public void onNewMessage(suturo_perception_msgs.ObjectDetection message) {
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
    
	    // start thread that reads the detections and adds them to KnowRob
		updateKnowRobObjDetections = new Thread( new UpdateKnowrobThread() );
		updateKnowRobObjDetections.start();
	}
	
	/**
	 * Read perceptions from the QueueingCallback buffer and create the
	 * corresponding object representations in KnowRob.
	 *
	 * @author 
	 *
	 */
	public class UpdateKnowrobThread implements Runnable {

		HashMap<String,ObjectDetection> objectsMap = new HashMap(); 
		
		@Override 
		public void run() {

			try {
				
				node.executeCancellableLoop(new CancellableLoop() {
					
					@Override
					protected void loop() throws InterruptedException {
						
						ObjectDetection oldObj;
						ObjectDetection obj = detections.take();
						if (!objectsMap.containsKey(obj.getName())){
							objectsMap.put(obj.getName(), obj);
						}
						else {
							oldObj = objectsMap.put(obj.getName(), obj);
							//now = obj.getPose().getHeader().getStamp();
								
							Matrix4d p = quaternionToMatrix(oldObj.getPose().getPose());					
							String q = "perceive_objects(" +
										"'http://knowrob.org/kb/knowrob.owl#"+oldObj.getName()+"', [" 
										+ p.m00 + "," + p.m01 + "," + p.m02 + "," + p.m03 + ","
										+ p.m10 + "," + p.m11 + "," + p.m12 + "," + p.m13 + ","
										+ p.m20 + "," + p.m21 + "," + p.m22 + "," + p.m23 + ","
										+ p.m30 + "," + p.m31 + "," + p.m32 + "," + p.m33 + "], " 
										+ oldObj.getType() + "," + oldObj.getWidth() + "," + oldObj.getHeight() 
										+ "," + oldObj.getDepth() + ", [" + oldObj.getPose().getHeader().getStamp() 
										+ ", " + obj.getPose().getHeader().getStamp() + "],  ObjInst)";
	
							// uncomment to see the resulting query printed to the KnowRob console
							//System.err.println(q);
							
							PrologInterface.executeQuery(q);
						}
					}
				});
				
			} catch (Exception e) { 
				// on program close, finish queries for leftovers of the hashmap
				e.printStackTrace();		
				for (Map.Entry<String, ObjectDetection> entry : objectsMap.entrySet()) {
					Matrix4d p = quaternionToMatrix(entry.getValue().getPose().getPose());	
					org.ros.message.Time now = node.getCurrentTime();
					String q = "perceive_objects(" +
								"'http://knowrob.org/kb/knowrob.owl#"+entry.getValue().getName() +"', [" 
								+ p.m00 + "," + p.m01 + "," + p.m02 + "," + p.m03 + ","
								+ p.m10 + "," + p.m11 + "," + p.m12 + "," + p.m13 + ","
								+ p.m20 + "," + p.m21 + "," + p.m22 + "," + p.m23 + ","
								+ p.m30 + "," + p.m31 + "," + p.m32 + "," + p.m33 + "], " 
								+ entry.getValue().getType() + "," + entry.getValue().getWidth() + "," 
								+ entry.getValue().getHeight() + "," + entry.getValue().getDepth() 
								+ ", [" + entry.getValue().getPose().getHeader().getStamp() + ", " 
								+ now  + "],  ObjInst)";

					// uncomment to see the resulting query printed to the KnowRob console
					//System.err.println(q);
					
					PrologInterface.executeQuery(q);
				}
			}
		}
	}
	
	/**
	 * Utility method: convert a ROS pose into a Java vecmath 4x4 pose matrix
	 *
	 * @param p Pose (ROS geometry_msgs)
	 * @return 4x4 pose matrix
	 */
	 public static Matrix4d quaternionToMatrix(Pose p) {

		return new Matrix4d(new Quat4d(p.getOrientation().getX(), p.getOrientation().getY(), p.getOrientation().getZ(), p.getOrientation().getW()), 
				new Vector3d(p.getPosition().getX(), p.getPosition().getY(), p.getPosition().getZ()), 1.0);
	}
	
}
