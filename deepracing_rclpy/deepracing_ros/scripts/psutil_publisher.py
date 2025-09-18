
import rclpy
import rclpy.node 
import rclpy.publisher  
import psutil
import std_msgs.msg

class PsutilPublisher(rclpy.node.Node):
    def __init__(self, name="psutil_publisher"):
        super(PsutilPublisher, self).__init__(name)
        self.cpuload_pub :rclpy.publisher.Publisher = self.create_publisher(std_msgs.msg.Float64, "cpu_percentage", 1)
        # Schedule the next call
        self.timer = self.create_timer(0.5, self.timer_callback, clock = self.get_clock())

    def timer_callback(self):
        cpu_percent = psutil.cpu_percent(interval=None)
        if (cpu_percent is None) or cpu_percent<=0.0: return
        cpu_msg = std_msgs.msg.Float64(data=cpu_percent)
        self.cpuload_pub.publish(cpu_msg)
        # self.get_logger().info(f"Published CPU load: {cpu_percent}%")




        

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = PsutilPublisher()
    rclpy.spin(node)
if __name__ == '__main__':
    main()