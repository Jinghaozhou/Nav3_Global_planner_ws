from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def to_path_message(node, trajectory, frame_id="map"):
    message = Path()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = frame_id
    for point in trajectory:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message
