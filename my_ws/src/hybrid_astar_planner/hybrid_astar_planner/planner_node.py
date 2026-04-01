import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from hybrid_astar_planner.hybrid_astar import HybridAStar

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # TF2 Setup: Initialize Buffer with the node to sync sim_time
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.planner = HybridAStar()
        self.map_data = None

    def map_cb(self, msg): 
        self.map_data = msg

    def get_robot_pose(self):
        try:
            # Using rclpy.time.Time() with NO arguments is the 
            # most reliable way to get "Latest Available" in ROS 2.
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time(), # Latest available
                timeout=rclpy.duration.Duration(seconds=0.5)) # Longer timeout
            
            return (trans.transform.translation.x, 
                    trans.transform.translation.y, 
                    0.0)
        except Exception as e:
            # If TF fails, we can't plan.
            self.get_logger().info(f'TF lookup failing: {e}')
            return None

    def goal_cb(self, msg):
        if self.map_data is None:
            self.get_logger().error("No map received!")
            return

        start = self.get_robot_pose()
        if start is None: return

        # GRID DEBUGGER: Let's see what the robot is standing on
        res = self.map_data.info.resolution
        ox = self.map_data.info.origin.position.x
        oy = self.map_data.info.origin.position.y
        w = self.map_data.info.width
        
        gx = int((start[0] - ox) / res)
        gy = int((start[1] - oy) / res)
        
        if 0 <= gx < w and 0 <= gy < self.map_data.info.height:
            val = self.map_data.data[gy * w + gx]
            self.get_logger().info(f"Robot at ({start[0]:.2f}, {start[1]:.2f}) -> Grid[{gx}, {gy}] = Val: {val}")
        else:
            self.get_logger().warn(f"Robot is OUTSIDE map bounds! Start: {start[:2]}")

        goal = (msg.pose.position.x, msg.pose.position.y, 0.0)
        self.get_logger().info(f"Planning Request: Goal {goal[:2]}")
        
        raw_path = self.planner.plan(
            start, goal, self.map_data.data, 
            w, self.map_data.info.height, ox, oy, res
        )

        if raw_path:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for p in raw_path:
                pose = PoseStamped()
                pose.pose.position.x, pose.pose.position.y = p[0], p[1]
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
            self.get_logger().info("SUCCESS: Path found!")
        else:
            self.get_logger().error("PLANNER FAIL: No path found. Check if goal is in a wall.")
            
def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()