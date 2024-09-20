from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class SubtractTwoIntsService(Node):

    def __init__(self):
        super().__init__('subtract_two_ints_service')
        self.srv = self.create_service(AddTwoInts, 'subtract_two_ints', self.subtract_two_ints_callback)

    def subtract_two_ints_callback(self, request, response):
        response.sum = request.a - request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    subtract_two_ints_service = SubtractTwoIntsService()

    rclpy.spin(subtract_two_ints_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()