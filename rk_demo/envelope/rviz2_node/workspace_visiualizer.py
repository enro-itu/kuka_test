import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import csv
import os

class WorkspaceVisualizer(Node):
    def __init__(self):
        super().__init__('workspace_visualizer')
        
        # Create the publisher as shown in your template
        self.publisher_ = self.create_publisher(MarkerArray, "/workspace_markers", 10)
        
        # Timer to publish data every 1 second
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        # CHANGE THIS to the actual path of your csv file
        self.csv_file_path = '/home/arifey/rk_ws/src/rk_demo/data/workspace_points.csv'
        
        # Store the marker internally so we don't re-read CSV every time
        self.marker_array = self.create_marker_array()

    def create_marker_array(self):
            marker_array = MarkerArray()
            
            marker = Marker()
            marker.header.frame_id = "map"  # Make sure this matches your RViz Fixed Frame
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.id = 0
            
            # 6 million points is dense, so let's make the dots smaller
            marker.scale.x = 0.01 
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            try:
                with open(self.csv_file_path, 'r') as f:
                    reader = csv.reader(f)
                    
                    # --- CHANGE STARTS HERE ---
                    for i, row in enumerate(reader):
                        # Skip empty lines
                        if not row: continue 
                        
                        # DOWNSAMPLING: If there is to much point to handle for Rviz2, reduce it with changing value i% 
                        if i % 1 != 0:
                            continue

                        try:
                            p = Point()
                            p.x = float(row[0])
                            p.y = float(row[1])
                            p.z = float(row[2])
                            marker.points.append(p)
                        except ValueError:
                            continue
                    # --- CHANGE ENDS HERE ---
                            
                self.get_logger().info(f"Visualizing {len(marker.points)} sampled points.")
                
            except FileNotFoundError:
                self.get_logger().error(f"Could not find file: {self.csv_file_path}")

            marker_array.markers.append(marker)
            return marker_array

    def publish_markers(self):
        # Update timestamp to current time
        if self.marker_array.markers:
            self.marker_array.markers[0].header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
