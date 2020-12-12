# https://answers.ros.org/question/314607/spawn-model-with-ros2-gazebo/
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

# arg O = x
# arg 1 = y
# arg 2 = z
def main():

    argv = sys.argv[1:]

    # Start node
    rclpy.init()
    node = rclpy.create_node("entity_spawner")
    node.get_logger().info('Creating Service client to connect to `/spawn_entity`')

    client = node.create_client(SpawnEntity, "/spawn_entity")
    node.get_logger().info("Connecting to `/spawn_entity` service...")

    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    sdf_file_path = os.path.join(get_package_share_directory("vacuumcleaner"), "robot.urdf")

    # Set data for request
    request = SpawnEntity.Request()
    request.name = "spawn entity"
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = "vacuumcleaner"
    request.initial_pose.position.x = float(argv[0])
    request.initial_pose.position.y = float(argv[1])
    request.initial_pose.position.z = float(argv[2])

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError('exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()