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

robot_name = "robot1"
broker="localhost"
port=1883
keepalive=60

class MQTTClient():

    def on_subscribe(client, userdata, mid, granted_qos):
        print("Subscribed to topic : " + str(mid) + " with Qos " + str(granted_qos))
        pass

    def on_publish(client, userdata, mid):
        print("Published to topic: " + str(mid) + "\n")
        pass

    def on_connect(client, userdata, flags, rc):
        print("Connected to MQTT broker with result code: " + str(rc) + "\n")
        pass 
    
    def on_disconnect(client, userdata, rc):
        if rc!=0:
            print("Unexpected disconnection") 
            pass
    client = mqtt.Client(robot_name, clean_session= False, userdata=None)
    client.on_publish= on_publish
    client.on_subscribe= on_subscribe
    client.on_connect= on_connect
    client.on_disconnect= on_disconnect
    client.username_pw_set("robot", "1234")
    client.reconnect_delay_set(min_delay=1, max_delay=6000)
    client.connect(broker, port, keepalive)
    

class PublishScan(MQTTClient):
    def __init__(self, frequency=10000):
        MQTTClient()
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        #rospy.Subscriber("tf_static", TransformStamped, self.tf_callback)
        self.scan_range = None
        self.pose = None
        self.transform_data = None
        self.last_published = rospy.get_time()
        self.interval = 0 if frequency is None else 1.0 / frequency
        self.tf2_listener()

    def tf2_listener(self):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():  
            try:
                trans = tfBuffer.lookup_transform('base_link','base_scan', rospy.Time())
                self.transform_data = trans
                print("trans")
                print(trans)
                rospy.logdebug("ROS tf data received from {}".format("base_scan to base_link"))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
    def base_scan_pose(self):
        base_scan_position = {"translation":{} , "rotation":{}}
        pose_transformed = tf2_geometry_msgs.do_transform_pose(self.pose, self.transform_data)
        print(pose_transformed)
        for i in ['x','y','z']:
            # exec(self.pose.pose.position."+ i +"+ self.transform_data.translation."+ i)
            # exec("self.pose.pose.orientation."+ i +"+ self.transform_data.rotation."+ i)
            base_scan_position["translation"][i] = getattr(pose_transformed.pose.position,i)
        
            
        for u in ['x','y','z','w']:
            base_scan_position["rotation"][u] = getattr(pose_transformed.pose.orientation,u)
        return base_scan_position

    def scan_callback(self,data):
        self.scan_range = json_message_converter.convert_ros_message_to_json(data)
        #print(self.scan_range)
        rospy.logdebug("ROS received from {}".format("scan"))
        self.mqtt_callback()
    def pose_callback(self,data):
        self.pose = data.pose
        rospy.logdebug("ROS received from {}".format("amcl_pose"))
    # def tf_callback(self,data):
    #     #if data.transforms[child_frame_id] == "base_scan":
    #     print(data.child_frame_id)
    #     #self.transform_data = tf.transform
    def mqtt_callback(self):
        if self.scan_range and self.pose and self.transform_data:
            now = rospy.get_time()
            if now - self.last_published >= self.interval:
                self.base_scan_position = self.base_scan_pose()
                print(self.base_scan_position)
                #print(type(self.scan_range),type(self.pose),type(self.transform_data))
                transform_data_json = self.convert_to_json(self.base_scan_position)
                scan_data_json = self.convert_to_json(self.scan_range)
                pkg_data = self.convert_to_json({"transform":transform_data_json  , "scan":self.scan_range})

            #print(transform_data_json)
                MQTTClient.client.publish("%s/scan"%(robot_name),pkg_data)
                #MQTTClient.client.publish("%s/scan"%(robot_name),scan_data_json)
                self.last_published = now
        else:
            print("no topics involved")

    def convert_to_json(self,data):
        y = yaml.load(str(data))
        return json.dumps(y, indent=None)
    
       

if __name__ == "__main__":
    rospy.init_node('publish_scan')
    publishscan = PublishScan()
    rospy.spin()


