import rclpy
from pymycobot.elephantrobot import ElephantRobot
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import socket
import numpy as np


class Cobot_Subscriber(Node):
    def __init__(self):
        super().__init__("Cobot_Subscriber")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.dummy_callback,
            100
        )
        self.subscription
     
        self.declare_parameter('ip', '192.168.1.159')
        self.declare_parameter('port', 5001)
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.get_logger().info("ip:%s, port:%d" % (ip, port))
        self.mc = ElephantRobot(ip, port)

        res = self.mc.start_client()
        if not res:
            print('res:', res)
            sys.exit(1)

        self.mc.set_speed(150)
    
    def dummy_callback(self, msg):
        # SERVER_IP = "192.168.1.159"
        # SERVER_PORT = 5001
        # MESSAGE = [message for message in msg.position]
        # MESSAGE[1] = -np.pi/2 + MESSAGE[1]
        # MESSAGE[3] = -np.pi/2 + MESSAGE[3]
        # message = "set_angles("
        # for i in range(len(MESSAGE)):
        #     message = message + (str(round(180/np.pi*MESSAGE[i]))) + ", " 
        # message = message+"500)"
        # print(message)
        # self.send_tcp_packet(SERVER_IP, SERVER_PORT, message)
        pass

    def send_tcp_packet(self, server_ip, server_port, message):
        try:
            # Create a TCP socket
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # Connect to the server
            client_socket.connect((server_ip, server_port))
            print(f"Connected to {server_ip}:{server_port}")

            # Send the message
            client_socket.sendall(message.encode('utf-8'))
            print(f"Sent: {message}")

            # Optionally receive a response (if server sends one)
            response = client_socket.recv(1024).decode('utf-8')
            print(f"Received: {response}")

        except socket.error as e:
            print(f"Error: {e}")

        finally:
            # Close the connection
            client_socket.close()


def main(args=None):
    rclpy.init(args=args)
    subscriber = Cobot_Subscriber()
    
    rclpy.spin(subscriber)
    
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()