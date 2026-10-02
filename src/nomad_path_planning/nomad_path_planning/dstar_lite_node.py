from nomad_path_planning.contracts import valid_grid
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String

from nomad_path_planning.dstar_lite import DStarLite


class DStarLiteNode(Node):
    def __init__(self):
        super().__init__('dstar_lite_gpp')

        self.planning_frame = self.declare_parameter('planning_frame', 'odom').value
        self.costmap = None
        self.current_pose = None
        self.goal_pose = None

        self.planner = None

        self.last_map_signature = None

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/nomad/costmap',
            self.costmap_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/nomad/current_pose',
            self.pose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/nomad/goal',
            self.goal_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.path_pub = self.create_publisher(
            Path,
            '/nomad/global_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.status_pub = self.create_publisher(
            String,
            '/nomad/gpp_status',
            10
        )

        self.get_logger().info(
            'D* Lite GPP node started'
        )

    def costmap_callback(self, msg):
        if not valid_grid(msg,self.planning_frame):
            self.costmap = None
            self.planner = None
            self.publish_empty_path()
            self.publish_status('GPP_WAIT: invalid costmap contract')
            return
        self.costmap = msg
        self.try_plan()

    def pose_callback(self, msg):
        self.current_pose = msg
        self.try_plan()

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.try_plan()

    def map_signature(self):
        return (
            self.costmap.info.width,
            self.costmap.info.height,
            self.costmap.info.resolution,
            self.costmap.info.origin.position.x,
            self.costmap.info.origin.position.y,
        )

    def world_to_cell(self, pose):
        resolution = self.costmap.info.resolution

        origin_x = (
            self.costmap.info.origin.position.x
        )

        origin_y = (
            self.costmap.info.origin.position.y
        )

        x = int(
            (
                pose.pose.position.x
                - origin_x
            )
            / resolution
        )

        y = int(
            (
                pose.pose.position.y
                - origin_y
            )
            / resolution
        )

        return x, y

    def cell_to_world(self, cell):
        x, y = cell

        resolution = self.costmap.info.resolution

        origin_x = (
            self.costmap.info.origin.position.x
        )

        origin_y = (
            self.costmap.info.origin.position.y
        )

        wx = (
            origin_x
            + (x + 0.5) * resolution
        )

        wy = (
            origin_y
            + (y + 0.5) * resolution
        )

        return wx, wy

    def publish_status(self, text):
        msg = String()
        msg.data = text

        self.status_pub.publish(msg)

    def publish_empty_path(self):
        msg = Path()
        msg.header.frame_id = self.planning_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(msg)

    def try_plan(self):
        if (
            self.costmap is None
            or self.current_pose is None
            or self.goal_pose is None
        ):
            return

        start = self.world_to_cell(
            self.current_pose
        )

        goal = self.world_to_cell(
            self.goal_pose
        )

        width = self.costmap.info.width
        height = self.costmap.info.height

        def valid(cell):
            return (
                0 <= cell[0] < width
                and 0 <= cell[1] < height
            )

        if not valid(start):
            self.get_logger().error(
                f'Start outside map: {start}'
            )
            self.publish_empty_path()
            return

        if not valid(goal):
            self.get_logger().error(
                f'Goal outside map: {goal}'
            )
            self.publish_empty_path()
            return

        signature = self.map_signature()

        rebuild = False

        if self.planner is None:
            rebuild = True

        elif signature != self.last_map_signature:
            rebuild = True

        elif goal != self.planner.goal:
            rebuild = True

        changed_cells = []

        if rebuild:

            self.planner = DStarLite(
                width=width,
                height=height,
                data=self.costmap.data,
                start=start,
                goal=goal,
            )

            self.last_map_signature = signature

            success = (
                self.planner.compute_shortest_path()
            )

            reason = 'INITIAL PLAN'

        else:
            start_moved = start != self.planner.start
            if start_moved:
                self.planner.move_start(start)

            changed_cells = (
                self.planner.update_costmap(
                    self.costmap.data
                )
            )

            if (
                not changed_cells
                and not start_moved
            ):
                return

            success = (
                self.planner.compute_shortest_path()
            )

            reason = (
                f'INCREMENTAL REPLAN '
                f'({len(changed_cells)} changed cells)'
            )

        if not success:
            self.get_logger().error(
                'D* Lite: path not found'
            )

            self.publish_status(
                'GPP_FAILED'
            )
            self.publish_empty_path()

            return

        path_cells = (
            self.planner.extract_path()
        )

        if not path_cells:
            self.get_logger().error(
                'D* Lite: path extraction failed'
            )

            self.publish_status(
                'GPP_FAILED'
            )
            self.publish_empty_path()

            return

        self.publish_path(
            path_cells
        )

        cost = self.planner.path_cost(
            path_cells
        )

        expansions = (
            self.planner.last_compute_expansions
        )

        text = (
            f'{reason}: '
            f'{len(path_cells)} cells, '
            f'cost={cost:.2f}, '
            f'expansions={expansions}'
        )

        self.get_logger().info(text)

        self.publish_status(text)

    def publish_path(self, path_cells):
        msg = Path()

        msg.header.frame_id = self.planning_frame
        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        for i, cell in enumerate(path_cells):

            pose = PoseStamped()

            pose.header = msg.header

            wx, wy = self.cell_to_world(
                cell
            )

            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0

            if i < len(path_cells) - 1:

                nx, ny = self.cell_to_world(
                    path_cells[i + 1]
                )

                yaw = math.atan2(
                    ny - wy,
                    nx - wx
                )

            else:
                yaw = 0.0

            pose.pose.orientation.z = (
                math.sin(yaw / 2.0)
            )

            pose.pose.orientation.w = (
                math.cos(yaw / 2.0)
            )

            msg.poses.append(pose)

        self.path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = DStarLiteNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
