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


class ReceiveScan():
    def __init__(self):
        #rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.pose = None
        pass
    
    # def conv_msg_pose(self,msg):
    #     pub = rospy.Publisher('%s/pose'%(subscribed_robot), LaserScan, queue_size=10)
    #     msg_trans = msg["translation"]
    #     msg_rot = msg["rotation"]
    #     msg_pose = PoseWithCovarianceStamped()
    #     msg_pose.header.frame_id = "%s/base_scan"%(subscribed_robot)
    #     msg_pose.pose.pose.position.x = msg_trans["x"]
    #     msg_pose.pose.pose.position.y = msg_trans["y"]
    #     msg_pose.pose.pose.position.z = msg_trans["z"]
    #     msg_pose.pose.pose.orientation.x = msg_rot["x"]
    #     msg_pose.pose.pose.orientation.y = msg_rot["y"]
    #     msg_pose.pose.pose.orientation.z = msg_rot["z"]
    #     msg_pose.pose.pose.orientation.w = msg_rot["w"]

    def transform_scan(self,msg):
        print(type(msg["translation"]["x"]))
        br = tf2_ros.TransformBroadcaster()     
        t = TransformStamped()  
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "%s/base_scan"%(subscribed_robot)
        msg_trans = msg["translation"]
        msg_rot = msg["rotation"]
        # q1 = [0,0,0,0]
        # q2 = [0,0,0,0]
        # print("biu")
        # q1[0]= msg_rot["x"]
        # q1[1] = msg_rot["y"]
        # q1[2] = msg_rot["z"]
        # q1[3] = msg_rot["w"]
        # print("ha")
        # q2[0] = self.pose.pose.orientation.x 
        # q2[1] = self.pose.pose.orientation.y
        # q2[2] = self.pose.pose.orientation.z 
        # q2[3] = self.pose.pose.orientation.w
        # print("haha") 
        # q_new = quaternion_multiply(q1,q2)
        # print(q1[2],q2[2],q_new[2])
        t.transform.translation.x = msg_trans["x"] 
        t.transform.translation.y = msg_trans["y"]
        t.transform.translation.z = msg_trans["z"]
        t.transform.rotation.x = msg_rot["x"]
        t.transform.rotation.y = msg_rot["y"]
        t.transform.rotation.z = msg_rot["z"]
        t.transform.rotation.w = msg_rot["w"]
        br.sendTransform(t)

    def publish_scan(self,msg):
        print("haha")
        pub = rospy.Publisher('%s/scan'%(subscribed_robot), LaserScan, queue_size=1000)
        #rate = ros
        # py.Rate(10)
        print("hoho")
        #while not rospy.is_shutdown():
        scan_msg = LaserScan()
        #scan_msg = message_converter.convert_dictionary_to_ros_message("sensor_msgs/LaserScan", msg)
        scan_msg.header.frame_id = "%s/base_scan"%(subscribed_robot)
        scan_msg.angle_increment= msg["angle_increment"]
        scan_msg.angle_max = msg["angle_max"]
        scan_msg.angle_min = msg["angle_min"]
        scan_msg.range_max = msg["range_max"]
        scan_msg.range_min = msg["range_min"]
        scan_msg.time_increment = msg["time_increment"]
        scan_msg.ranges = msg["ranges"]
        scan_msg.intensities = msg["intensities"]
        #print(msg["scan"])
        #print(scan_msg)
        pub.publish(scan_msg)
        rospy.logdebug("ROS published to {}/scan".format(subscribed_robot))
        #rate.sleep()
    
    # def pose_callback(self,data):
    #     self.pose = data.pose
    #     rospy.logdebug("ROS received from {}".format("amcl_pose"))


class MQTTClient(ReceiveScan):
    
    def __init__(self,client):
        ReceiveScan.__init__(self)
        client.on_publish= self.on_publish
        client.on_subscribe= self.on_subscribe
        client.on_connect= self.on_connect
        client.on_disconnect= self.on_disconnect
        client.on_message = self.on_message
        self.count = 0
        self.now = time.time()
        self.frequency = 0
        client.username_pw_set(username,password)
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

    def on_message(self,client, userdata, msg): 
        print("Received message: on topic " + str(msg.topic) + " " + "with QoS " + str(msg.qos))
        self.count+= 1
        print(self.frequency)
        if time.time() - self.now >=60:
            self.frequency = self.count
            self.count = 0
            self.now = time.time()
        m_in = msg.payload.decode("utf-8","ignore")
        m_json = json.loads(m_in)
        m_scan_json = json.loads(m_json["scan"])
        m_transform_json = json.loads(m_json["transform"])
        print(type(m_transform_json))
        ReceiveScan.transform_scan(self,m_transform_json)
        ReceiveScan.publish_scan(self,m_scan_json)

        

if __name__ == "__main__":
    rospy.init_node('receive_scan')
    robot_name =  rospy.get_param("/robot_name", 'robot2')
    subscribed_robot = rospy.get_param("/subscribe_robot",'robot1')
    broker=rospy.get_param("/mqtt_broker", 'localhost')
    port=rospy.get_param("/port", 1883)
    username=rospy.get_param("/username", 'robot')
    password=rospy.get_param("/password", '1234')
    keepalive=60
    client = mqtt.Client("%s/subscriber"%(robot_name), clean_session= False, userdata=None)
    mqtt_start = MQTTClient(client)
    rospy.spin()
