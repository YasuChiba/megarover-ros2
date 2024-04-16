import sys
import csv
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QFileDialog
from geometry_msgs.msg import PoseStamped

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def publish_pose(self, x, y):
        msg = PoseStamped()
        # add timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher.publish(msg)

        print(f"Published goal pose at x={x}, y={y}")

class AppWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()

    def initUI(self):
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.setWindowTitle('Goal Pose Setter')
        self.setGeometry(300, 300, 300, 250)

        # Button to load CSV
        self.load_button = QPushButton('Load CSV', self)
        self.load_button.clicked.connect(self.loadCSV)
        self.layout.addWidget(self.load_button)

        self.show()

    def loadCSV(self):
        fname, _ = QFileDialog.getOpenFileName(self, 'Open CSV file', '.', "CSV files (*.csv)")
        if fname:
            self.createButtonsFromCSV(fname)

    def createButtonsFromCSV(self, file_path):
        with open(file_path, mode='r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                # Format coordinates to 4 decimal places
                x = float(row['X Coordinate'])
                y = float(row['Y Coordinate'])
                label = row['Label']
                formatted_label = f"{label} ({x:.2f}, {y:.2f})"
                button = QPushButton(formatted_label)
                button.clicked.connect(lambda ch, x=x, y=y: self.node.publish_pose(x, y))
                self.layout.addWidget(button)


def main():
    # Initialize ROS 2
    rclpy.init(args=None)
    node = GoalPosePublisher()
    
    # Start the Qt application
    app = QApplication(sys.argv)
    ex = AppWindow(node)
    result = app.exec_()
    
    # Shutdown ROS 2
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(result)

if __name__ == '__main__':
    main()
