#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import json
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped, Quaternion
import paho.mqtt.client as mqtt
from rospy_message_converter import message_converter
import numpy as np
import tf2_geometry_msgs
from tf.transformations import *
import time

#create receive scan class that receives mqtt scan emssage and publish to ros
class ReceiveScan():
    def __init__(self):
        #init class variables
        self.pose = None
        pass

    #transform laser data to map frame with tf2
    def transform_scan(self,msg):
        print(type(msg["translation"]["x"]))
        br = tf2_ros.TransformBroadcaster()     
        t = TransformStamped()  
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "%s/base_scan"%(subscribed_robot)
        msg_trans = msg["translation"]
        msg_rot = msg["rotation"]
        #converting transform parameters with lidar poses transmitted through mqtt
        t.transform.translation.x = msg_trans["x"] 
        t.transform.translation.y = msg_trans["y"]
        t.transform.translation.z = msg_trans["z"]
        t.transform.rotation.x = msg_rot["x"]
        t.transform.rotation.y = msg_rot["y"]
        t.transform.rotation.z = msg_rot["z"]
        t.transform.rotation.w = msg_rot["w"]
        br.sendTransform(t)

    #publish scan data to robot_name/scan topic
    def publish_scan(self,msg):
        pub = rospy.Publisher('%s/scan'%(subscribed_robot), LaserScan, queue_size=1000)
        #rate = ros
        # py.Rate(10)
        #while not rospy.is_shutdown():
        scan_msg = LaserScan()
        scan_msg.header.frame_id = "%s/base_scan"%(subscribed_robot)
        scan_msg.angle_increment= msg["angle_increment"]
        scan_msg.angle_max = msg["angle_max"]
        scan_msg.angle_min = msg["angle_min"]
        scan_msg.range_max = msg["range_max"]
        scan_msg.range_min = msg["range_min"]
        scan_msg.time_increment = msg["time_increment"]
        scan_msg.ranges = msg["ranges"]
        scan_msg.intensities = msg["intensities"]
        pub.publish(scan_msg)
        rospy.logdebug("ROS published to {}/scan".format(subscribed_robot))

#create MQTTClient class that subcribes to the lidar data from other robots
class MQTTClient(ReceiveScan):
    
    def __init__(self,client):
        ReceiveScan.__init__(self) #init ReceiveScan Class through class inheritance
        client.on_publish= self.on_publish
        client.on_subscribe= self.on_subscribe
        client.on_connect= self.on_connect
        client.on_disconnect= self.on_disconnect
        client.on_message = self.on_message
        self.count = 0
        self.now = time.time()
        self.frequency = 0 #flag that keep track of the message receive frequency for mqtt
        client.username_pw_set(username,password) #MQTT Broker password and username
        client.reconnect_delay_set(min_delay=1, max_delay=6000)
        client.connect(broker, port, keepalive)
        client.loop_start()
        

    def on_subscribe(self,client, userdata, mid, granted_qos):
        print("Subscribed to topic : " + str(mid) + " with Qos " + str(granted_qos))
        pass

    def on_publish(self,client, userdata, mid):
        print("Published to topic: " + str(mid) + "\n")
        pass

    def on_connect(self,client, userdata, flags, rc):
        print("Connected to MQTT broker with result code: " + str(rc) + "\n")
        client.subscribe("%s/scan"%(subscribed_robot),0)
        #client.subscribe("%s/transform"%(subscribed_robot),0)
    
    def on_disconnect(self,client, userdata, rc):
        if rc!=0:
            print("Unexpected disconnection") 
            pass

    #callback function for every message received from MQTT
    def on_message(self,client, userdata, msg): 
        print("Received message: on topic " + str(msg.topic) + " " + "with QoS " + str(msg.qos))
        self.count+= 1
        print(self.frequency)
        if time.time() - self.now >=60:
            self.frequency = self.count
            self.count = 0
            self.now = time.time()
        m_in = msg.payload.decode("utf-8","ignore")
        #convert json type data to dictionary python data structure
        m_json = json.loads(m_in)
        m_scan_json = json.loads(m_json["scan"])
        m_transform_json = json.loads(m_json["transform"])
        print(type(m_transform_json))
        ReceiveScan.transform_scan(self,m_transform_json)
        ReceiveScan.publish_scan(self,m_scan_json)

        

if __name__ == "__main__":
    rospy.init_node('receive_scan')
    ##### get parameters for parameter server #######
    robot_name =  rospy.get_param("/robot_name", 'robot2')
    subscribed_robot = rospy.get_param("/subscribe_robot",'robot1')
    broker=rospy.get_param("/mqtt_broker", 'localhost')
    port=rospy.get_param("/port", 1883)
    username=rospy.get_param("/username", 'robot')
    password=rospy.get_param("/password", '1234')
    keepalive=60
    ##### init mqtt client #######
    client = mqtt.Client("%s/subscriber"%(robot_name), clean_session= False, userdata=None)
    mqtt_start = MQTTClient(client)
    rospy.spin()
