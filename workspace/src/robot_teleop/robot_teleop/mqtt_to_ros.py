import json
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MqttToRos(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros')
        self.publisher_ = self.create_publisher(Twist, 'rover_twist', 10)
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect('mqtt_broker_address', 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info('Connected to MQTT broker')
        client.subscribe('mqtt_topic')

    def on_message(self, client, userdata, msg):
        self.get_logger().info(f'Received message: {msg.payload.decode()}')
        try:
            data = json.loads(msg.payload.decode())
            twist = Twist()
            twist.linear.x = data['linear']['x']
            twist.linear.y = data['linear']['y']
            twist.linear.z = data['linear']['z']
            twist.angular.x = data['angular']['x']
            twist.angular.y = data['angular']['y']
            twist.angular.z = data['angular']['z']
            self.publisher_.publish(twist)
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Failed to parse message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MqttToRos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()