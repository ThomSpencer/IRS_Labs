import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MyServiceClient(Node):
    def __init__(self):
        super().__init__('pineapple_box_sorter')
        self.cli = self.create_client(Trigger, 'my_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = MyServiceClient()
    node.send_request()
    node.get_logger().info('Sending request')

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info('Service call failed: %r' % (e,))
            else:
                node.get_logger().info('Service response: %s' % response.message)
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
