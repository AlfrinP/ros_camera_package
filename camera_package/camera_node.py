import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from glob import glob
from ament_index_python.packages import get_package_share_directory


class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__("image_publisher_node")

        # Create publisher
        self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Set the image directory and get list of images
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.get_logger().info(f"Current working directory: {os.getcwd()}")
        package_share_dir = get_package_share_directory("camera_package")
        self.image_dir = os.path.join(package_share_dir, "photos")
        self.get_logger().info(
            f"Image directory: {self.image_dir}"
        )  # Change this to your image directory
        self.image_files = []
        self.current_image_index = 0

        # Load image files
        self.load_images()

        # Create timer for 30 FPS
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info("Image Publisher Node started")

    def load_images(self):
        """Load all images from the directory"""
        try:
            # Get all image files (supporting common formats)
            self.image_files = sorted(
                glob(os.path.join(self.image_dir, "*.[jJ][pP][gG]"))
            )
            self.image_files.extend(
                sorted(glob(os.path.join(self.image_dir, "*.[pP][nN][gG]")))
            )
            self.image_files.extend(
                sorted(glob(os.path.join(self.image_dir, "*.[jJ][pP][eE][gG]")))
            )

            if not self.image_files:
                self.get_logger().error(f"No images found in {self.image_dir}")
                return

            self.get_logger().info(f"Found {len(self.image_files)} images")

        except Exception as e:
            self.get_logger().error(f"Error loading images: {e}")

    def timer_callback(self):
        """Publish image at 30 FPS"""
        if not self.image_files:
            self.get_logger().warn("No images available to publish")
            return

        try:
            # Read the current image
            image_path = self.image_files[self.current_image_index]
            cv_image = cv2.imread(image_path)

            if cv_image is None:
                self.get_logger().error(f"Failed to read image: {image_path}")
                return

            # Convert to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

            # Publish the image
            self.publisher.publish(ros_image)
            self.get_logger().info(f"Published image: {os.path.basename(image_path)}")

            # Move to next image, loop back to start if at end
            self.current_image_index = (self.current_image_index + 1) % len(
                self.image_files
            )

        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImagePublisherNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error running node: {e}")
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
