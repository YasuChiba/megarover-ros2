import json
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor  

class TeleopClient(Node):
    def __init__(self):
        super().__init__('teleop_client')
        self.publisher_ = self.create_publisher(Twist, '/rover_twist', 10)

        self.robotId = "robot1"
        self.teleopTopic = None

        host = '172.26.30.43'
        port = 1883
        username = self.robotId
        password = "robotpasswd"
        self.mqtt_client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.mqtt_client.username_pw_set(username, password=password)
        self.mqtt_client.on_connect = self.mqtt_on_connect
        self.mqtt_client.on_message = self.mqtt_on_message

        self.mqtt_client.connect(host, port=port, keepalive=60)
        self.mqtt_client.loop_start()

    def mqtt_on_connect(self, client, userdata, flags, rc):
        self.get_logger().info('Connected to MQTT broker')
        self.mqtt_client.subscribe(f"/teleop/elev/{self.robotId}/request/response")
        self.mqtt_client.subscribe(f"/teleop/elev/{self.robotId}/finished")


    def mqtt_on_message(self, client, userdata, msg: mqtt.MQTTMessage):

        if msg.retain:
            self.get_logger().info(f'Received retained message: {msg.payload.decode()}' + "   " + msg.topic)
            return
        
        # log topic and message
        self.get_logger().info(f'Received message: {msg.payload.decode()}' + "   " + msg.topic)

        if msg.topic == f"/teleop/elev/{self.robotId}/request/response":
            data = json.loads(msg.payload.decode())
            self.teleopTopic = data['topic_name']
            self.mqtt_client.subscribe(self.teleopTopic)
            self.get_logger().info(f'Starting teleop on topic {self.teleopTopic}')

        elif msg.topic == f"/teleop/elev/{self.robotId}/finished":
            self.mqtt_client.unsubscribe(self.teleopTopic)
            self.teleopTopic = None
            self.get_logger().info('Teleop finished')

        elif msg.topic == self.teleopTopic:
            data = json.loads(msg.payload.decode())
            twist = Twist()
            twist.linear.x = data['linear']['x']
            twist.linear.y = data['linear']['y']
            twist.linear.z = data['linear']['z']
            twist.angular.x = data['angular']['x']
            twist.angular.y = data['angular']['y']
            twist.angular.z = data['angular']['z']
            self.publisher_.publish(twist)

            self.get_logger().info(f'Publishing twist: {twist}')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()