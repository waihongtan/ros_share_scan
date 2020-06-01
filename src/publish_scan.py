#!/usr/bin/env python
import rospy    
from sensor_msgs.msg import LaserScan
#from tf2_msgs.msg import TransformStamped
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_geometry_msgs
import json
import yaml
import paho.mqtt.client as mqtt
from rospy_message_converter import message_converter, json_message_converter


#create MQTT Class for MQTT client setu[]
class MQTTClient():

    def __init__(self):
        global client
        client.on_publish= self.on_publish
        client.on_subscribe= self.on_subscribe
        client.on_connect= self.on_connect
        client.on_disconnect= self.on_disconnect
        client.username_pw_set(username, password) #username and password for mqtt broker authentication
        client.reconnect_delay_set(min_delay=1, max_delay=6000) #maximum delay for reconnect attempt
        client.connect(broker, port, keepalive)#specification of broker ip and port
        client.loop_start()

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed to topic : " + str(mid) + " with Qos " + str(granted_qos))
        #subcribe to topic callback
        pass

    def on_publish(self, client, userdata, mid):
        print("Published to topic: " + str(mid) + "\n")
        #published payload callback
        pass

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code: " + str(rc) + "\n")
        #connected to broker callback
        pass 
    
    def on_disconnect(self, client, userdata, rc):
        if rc!=0:
            print("Unexpected disconnection") 
            #disconnected callback
            pass
    
#create class for ROS Operations
class PublishScan(MQTTClient):
    def __init__(self, frequency=10000):
        MQTTClient.__init__(self) #init MQTT Class through class inheritance
        rospy.Subscriber("scan", LaserScan, self.scan_callback) #subscribed to scan topic from ros lidar driver
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.pose_callback) #subscribe to pose topic from ros nav stack
        #init variables
        self.scan_range = None 
        self.pose = None
        self.transform_data = None
        self.last_published = rospy.get_time()
        self.interval = 0 if frequency is None else 1.0 / frequency #interval for publishing scan message and transform coordinates through MQTT
        self.tf2_listener() #INIT Transform listener

    def tf2_listener(self):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():  
            try:
                trans = tfBuffer.lookup_transform('base_link','base_scan', rospy.Time()) #get transform information between base link and base_scan frame
                self.transform_data = trans 
                print("trans")
                print(trans)
                rospy.logdebug("ROS tf data received from {}".format("base_scan to base_link"))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
    #create function that generate the transformed pose with tf2 from base_link frame to base_scan fram       
    def base_scan_pose(self):
        base_scan_position = {"translation":{} , "rotation":{}}
        pose_transformed = tf2_geometry_msgs.do_transform_pose(self.pose, self.transform_data) #transformed pose of base_scan
        print(pose_transformed) 
        for i in ['x','y','z']:
            base_scan_position["translation"][i] = getattr(pose_transformed.pose.position,i) #append translation coordinates of base_scan pose
        
            
        for u in ['x','y','z','w']:
            base_scan_position["rotation"][u] = getattr(pose_transformed.pose.orientation,u) #append quartenion coordinates of base_scan pose
        return base_scan_position

    #callback for scan topic subscription
    def scan_callback(self,data):
        self.scan_range = json_message_converter.convert_ros_message_to_json(data) #convert ROS LaserScan class type message to json type
        #print(self.scan_range)
        rospy.logdebug("ROS received from {}".format("scan"))
        self.mqtt_callback()
    
    #callback for amcl pose subscription
    def pose_callback(self,data):
        self.pose = data.pose
        rospy.logdebug("ROS received from {}".format("amcl_pose"))

    #publish data to mqtt every scan message received
    def mqtt_callback(self):
        if self.scan_range and self.pose and self.transform_data:
            now = rospy.get_time()
            #create frequency interval for mqtt publish
            if now - self.last_published >= self.interval:
                self.base_scan_position = self.base_scan_pose()
                print(self.base_scan_position)
                #print(type(self.scan_range),type(self.pose),type(self.transform_data))

                #convert data type to json
                transform_data_json = self.convert_to_json(self.base_scan_position)
                scan_data_json = self.convert_to_json(self.scan_range)
                pkg_data = self.convert_to_json({"transform":transform_data_json  , "scan":self.scan_range})

            #print(transform_data_json)
                client.publish("%s/scan"%(robot_name),pkg_data)
                #MQTTClient.client.publish("%s/scan"%(robot_name),scan_data_json)
                self.last_published = now
        else:
            print("no topics involved")

    #function to convert data to json
    def convert_to_json(self,data):
        y = yaml.load(str(data))
        return json.dumps(y, indent=None)
    
       

if __name__ == "__main__":
    rospy.init_node('publish_scan')
    ##get parameters from ros parameters server ##
    robot_name =  rospy.get_param("/robot_name", 'robot1')
    broker= rospy.get_param("/mqtt_broker", 'localhost')
    port=rospy.get_param("/port", 1883)
    username =rospy.get_param("/username", 'robot')
    password =rospy.get_param("/password", '1234')
    ### init mosquitto client ##
    client = mqtt.Client("%s/publisher"%(robot_name), clean_session= False, userdata=None)
    keepalive=60
    ## init publish scan class##
    publishscan = PublishScan()
    rospy.spin()


