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

import org.knowrob.json_prolog.client.PrologClient;
import org.knowrob.json_prolog.PrologBindings;
import org.knowrob.json_prolog.client.PrologQueryProxy;
import org.ros.concurrent.CancellableLoop;
import org.knowrob.utils.ros.RosUtilities;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.knowrob.prolog.PrologInterface;


/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class Listener extends AbstractNodeMain {
	
	ConnectedNode node;
	BlockingQueue<ObjectDetection> detections;
	Thread updateKnowRobObjDetections;
	static Log log;

	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rosjava/listener");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		// save reference to the ROS node
		this.node = connectedNode;
		//init queue for detected objects
		this.detections = new LinkedBlockingQueue<ObjectDetection>();
		
		log = connectedNode.getLog();
		// wait for node to be ready
		try {
			while(node ==null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		//start subscriber for object detection
	    Subscriber<ObjectDetection> subscriber = connectedNode.newSubscriber("dummy_object_detections", ObjectDetection._TYPE);
	    subscriber.addMessageListener(new MessageListener<ObjectDetection>() {
	    	@Override
	    	//when new object detected in channel, put object in detection queue
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
		//start HashMap for detected objects
		HashMap<String,ObjectDetection> objectsMap = new HashMap(); 
		ObjectDetection oldObj, currObj = null;
		PrologClient pl = new PrologClient();
		ServiceClient<json_prolog_msgs.PrologQueryRequest, json_prolog_msgs.PrologQueryResponse> query_client;
		ServiceClient<json_prolog_msgs.PrologNextSolutionRequest, json_prolog_msgs.PrologNextSolutionResponse> next_solution_client;
		int id = 0;

		public boolean queryIt(String q){
			try {
				query_client = node.newServiceClient("json_prolog/simple_query", json_prolog_msgs.PrologQuery._TYPE);
				next_solution_client = node.newServiceClient("json_prolog/next_solution", json_prolog_msgs.PrologNextSolution._TYPE);

				final json_prolog_msgs.PrologQueryRequest req1 = query_client.newMessage();
				id++;
				req1.setId(id + "");
				req1.setQuery(q);

				query_client.call(req1, new ServiceResponseListener<json_prolog_msgs.PrologQueryResponse>() {

					@Override
					public void onSuccess(json_prolog_msgs.PrologQueryResponse response) {
						log.info("Query " + req1.getQuery() + "successful");
					}

					@Override
					public void onFailure(RemoteException e) {
						throw new RosRuntimeException(e);
					}
				});

						//FIX: Query the next solution
				final json_prolog_msgs.PrologNextSolutionRequest req2 = next_solution_client.newMessage();
				req2.setId(id + "");

				next_solution_client.call(req2, new ServiceResponseListener<json_prolog_msgs.PrologNextSolutionResponse>() {
					@Override
					public void onSuccess(json_prolog_msgs.PrologNextSolutionResponse response) {
						log.info("Query " + req2.getId() + "successful!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
					}

					@Override
					public void onFailure(RemoteException e) {
						throw new RosRuntimeException(e);
					}
				});

				// final json_prolog_msgs.PrologFinishRequest req3 = next_solution_client.newMessage();
				// req3.setId(id + "");

				// next_solution_client.call(req3, new ServiceResponseListener<json_prolog_msgs.PrologFinishResponse>() {
				// 	@Override
				// 	public void onSuccess(json_prolog_msgs.PrologFinishResponse response) {
				// 		log.info("Query " + req3.getId() + "successful!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				// 	}

				// 	@Override
				// 	public void onFailure(RemoteException e) {
				// 		throw new RosRuntimeException(e);
				// 	}
				// });
				return true;
			} catch (Exception e) {
				e.printStackTrace();
			}
			return false;
		}

		@Override 
		public void run() {
			try {				
				node.executeCancellableLoop(new CancellableLoop() {
										
					@Override
					protected void loop() throws InterruptedException {
						
						currObj = detections.take();
						//if object has not been seen before, start temporal representation
						if (!objectsMap.containsKey(currObj.getName())){
							objectsMap.put(currObj.getName(), currObj);
							
							Matrix4d p = quaternionToMatrix(currObj.getPose().getPose());

							//String format: perceive_objects(Name, PoseAsList, Type, Frame_id, Width, Height, Depth, Begin, ObjInst) :- 
							String q = "create_object_state('" +
										currObj.getName()+"', [" 
										+ p.m00 + "," + p.m01 + "," + p.m02 + "," + p.m03 + ","
										+ p.m10 + "," + p.m11 + "," + p.m12 + "," + p.m13 + ","
										+ p.m20 + "," + p.m21 + "," + p.m22 + "," + p.m23 + ","
										+ p.m30 + "," + p.m31 + "," + p.m32 + "," + p.m33 + "], " 
										+ currObj.getType() + ",'" + currObj.getPose().getHeader().getFrameId() + "',"
										+ currObj.getWidth() + "," + currObj.getHeight() + "," + currObj.getDepth() 
										+ ", [" + currObj.getPose().getHeader().getStamp() + "],  ObjInst)";


							queryIt(q);
							log.info(q);

						}
						//else (object has been seen before) close previous temporal rep and start new rep
						else {
							oldObj = objectsMap.put(currObj.getName(), currObj);
							//now = obj.getPose().getHeader().getStamp();
							//#######################################
							//Close previous Interval of Object
							
							Matrix4d p_old = quaternionToMatrix(oldObj.getPose().getPose());	
							
							//String format: perceive_objects(Name, PoseAsList, Type, Frame_id, Width, Height, Depth, [Begin, End], ObjInst) :- 
							String q_old = "create_object_state_with_close(" +
										oldObj.getName()+", [" 
										+ p_old.m00 + "," + p_old.m01 + "," + p_old.m02 + "," + p_old.m03 + ","
										+ p_old.m10 + "," + p_old.m11 + "," + p_old.m12 + "," + p_old.m13 + ","
										+ p_old.m20 + "," + p_old.m21 + "," + p_old.m22 + "," + p_old.m23 + ","
										+ p_old.m30 + "," + p_old.m31 + "," + p_old.m32 + "," + p_old.m33 + "], " 
										+ oldObj.getType() + "," + oldObj.getPose().getHeader().getFrameId() + ","
										+ oldObj.getWidth() + "," + oldObj.getHeight() + "," + oldObj.getDepth() 
										+ ", [" + oldObj.getPose().getHeader().getStamp() 
										+ ", " + currObj.getPose().getHeader().getStamp() + "],  ObjInst)";
	
							// uncomment to see the resulting query printed to the KnowRob console
							//System.err.println(q);
							queryIt(q_old);
				
							
							//#######################################
							//Open new Interval of Object
							
							//Matrix4d p_new = quaternionToMatrix(oldObj.getPose().getPose());

							//String format: perceive_objects(Name, PoseAsList, Type, Frame_id, Width, Height, Depth, Begin, ObjInst) :- 
							//String q_new = "create_object_state(" +
							//			"'http://knowrob.org/kb/knowrob.owl#"+currObj.getName()+"', [" 
							//			+ p_new.m00 + "," + p_new.m01 + "," + p_new.m02 + "," + p_new.m03 + ","
							//			+ p_new.m10 + "," + p_new.m11 + "," + p_new.m12 + "," + p_new.m13 + ","
							//			+ p_new.m20 + "," + p_new.m21 + "," + p_new.m22 + "," + p_new.m23 + ","
							//			+ p_new.m30 + "," + p_new.m31 + "," + p_new.m32 + "," + p_new.m33 + "], " 
							//			+ currObj.getType() + "," + currObj.getPose().getHeader().getFrameId() + ","
							//			+ currObj.getWidth() + "," + currObj.getHeight() + "," + currObj.getDepth() 
							//			+ ", " + currObj.getPose().getHeader().getStamp() + ",  ObjInst)";
							
							//PrologInterface.executeQuery(q_new);

						}
					}
				});
				
			} catch (Exception e) { 
				// on program close, finish queries for leftovers of the hashmap
				e.printStackTrace();		
				for (Map.Entry<String, ObjectDetection> entry : objectsMap.entrySet()) {
					Matrix4d p = quaternionToMatrix(entry.getValue().getPose().getPose());	
					org.ros.message.Time now = node.getCurrentTime();
					String q = "create_object_state_with_close(" +
								entry.getValue().getName() +", [" 
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
					queryIt(q);
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
