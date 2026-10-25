#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan


class LaserScanFilter(Node):
    """
    Subscribes to raw LaserScan (e.g. /scan) and filters out self-reflections
    caused by robot chassis/caster/wheel geometry in specified angular sectors.
    Publishes cleaned data to /scan_filtered.
    """

    def __init__(self):
        super().__init__('scan_filter')

        # Declare configurable parameters
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        # Flattened pairs of [min_deg, max_deg, min_deg, max_deg, ...]
        self.declare_parameter(
            'filter_sectors',
            [-150.0, -143.0, -36.0, -30.0, 30.0, 36.0, 143.0, 151.0]
        )
        # Maximum distance (meters) to filter within the sectors.
        # Self-reflections occur at 0.22-0.28m. Real obstacles beyond 0.35m are preserved.
        # Set to <= 0.0 or inf to unconditionally filter all distances in the sector.
        self.declare_parameter('sector_max_range', 0.35)
        self.declare_parameter('min_range_cutoff', 0.0)
        self.declare_parameter('replacement_value', 'inf')

        # Read parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        raw_sectors = self.get_parameter('filter_sectors').get_parameter_value().double_array_value
        self.sector_max_range = self.get_parameter('sector_max_range').get_parameter_value().double_value
        self.min_range_cutoff = self.get_parameter('min_range_cutoff').get_parameter_value().double_value
        rep_str = self.get_parameter('replacement_value').get_parameter_value().string_value.lower()

        # Parse angle sectors into list of (min_deg, max_deg) tuples
        self.sectors = []
        for i in range(0, len(raw_sectors), 2):
            if i + 1 < len(raw_sectors):
                s_min = raw_sectors[i]
                s_max = raw_sectors[i + 1]
                self.sectors.append((min(s_min, s_max), max(s_min, s_max)))

        # Determine replacement float value
        if rep_str == 'nan':
            self.replacement_val = float('nan')
        else:
            self.replacement_val = float('inf')

        self.get_logger().info(
            f"LaserScanFilter initialized: '{input_topic}' -> '{output_topic}' | "
            f"Sectors (deg): {self.sectors} | Max filter dist: {self.sector_max_range}m"
        )

        # Subscriber & Publisher with sensor_data QoS (Best Effort)
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data
        )

    def scan_callback(self, msg: LaserScan):
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        ranges = list(msg.ranges)
        num_readings = len(ranges)

        for i in range(num_readings):
            r = ranges[i]

            # Calculate beam angle in radians and normalize to [-pi, pi]
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            angle_deg = math.degrees(angle_rad)

            # Check if this beam falls inside any filtering sector
            in_sector = False
            for s_min, s_max in self.sectors:
                if s_min <= angle_deg <= s_max:
                    in_sector = True
                    break

            if in_sector:
                # If sector_max_range <= 0 or inf, unconditionally filter.
                # Otherwise, only filter if the reading is within the self-reflection distance.
                if self.sector_max_range <= 0.0 or math.isinf(self.sector_max_range) or r <= self.sector_max_range:
                    ranges[i] = self.replacement_val
            elif self.min_range_cutoff > 0.0 and r < self.min_range_cutoff:
                ranges[i] = self.replacement_val

        filtered_msg.ranges = ranges
        if msg.intensities:
            filtered_msg.intensities = list(msg.intensities)

        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
