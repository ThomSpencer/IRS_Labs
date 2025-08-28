import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String

class PlcHmiListener(Node):
    def __init__(self):
        super().__init__('plc_hmi_listener') # Node name
        # Create publisher: topic_name, message_type, queue_size
        self.subscription_ = self.create_subscription(String, '/hmi/unified_status',self.listener_callback,1)
        self.publisher_ = self.create_publisher(String, 'status_updates',1)

    def listener_callback(self,msg: String):
        try:
            data = json.loads(msg.data) # parse JSON string into Python dict
            stamp = data["stamp"]
            box = data["box"]
            counts = data["counts"]
            
            msgSend = String()
            msgSend = ("Received PLC status:\n")
            msgSend += (f"Time: {stamp['sec']}\n")
            msgSend += (f"Box weight raw={box['weight_raw']}\n")
            msgSend += (f"Location: {box['location']}\n")
            msgSend += (f"Counts: big={counts['big']}, medium={counts['medium']}, small={counts['small']}, total={counts['total']}\n")
            
            self.publisher_.publish(String(data=msgSend))
            #self.get_logger().info(f'Publishing: "{msg.data}"')
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}\nRaw msg={msg.data}")

def main(args=None):
    rclpy.init(args=args) # Initialize ROS2
    plchmilistener = PlcHmiListener()
    rclpy.spin(plchmilistener) # Keep node running
    plchmilistener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()