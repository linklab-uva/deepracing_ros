
#!/usr/bin/env python3

import time
import rclpy
import rclpy.node 
import rclpy.publisher  
import psutil
import std_msgs.msg

class PsutilPublisher(rclpy.node.Node):
    def __init__(self, name="psutil_publisher"):
        super(PsutilPublisher, self).__init__(name)
        self.cpuload_pub : rclpy.publisher.Publisher = self.create_publisher(std_msgs.msg.Float64, "cpu_percentage", 1)
        self.cpuload_perthread_pub : rclpy.publisher.Publisher = self.create_publisher(std_msgs.msg.Float64, "cpu_percentage_perthread", 1)
        self.memory_pub : rclpy.publisher.Publisher = self.create_publisher(std_msgs.msg.Float64, "memory_percentage", 1)
        self.nthreads_pub : rclpy.publisher.Publisher = self.create_publisher(std_msgs.msg.Int32, "Nthreads", 1)
        process_name_param = self.declare_parameter("process_name", value="")

        process_name = process_name_param.get_parameter_value().string_value
        self.process : psutil.Process | None = None
        if (process_name is None) or (len(process_name.strip())==0):
            self.get_logger().info("No process name specified, publishing system-wide CPU and memory usage.")
        else:
            attempts = 10
            while self.process is None and attempts>0:
                attempts -= 1
                self.get_logger().info(f"Looking for process with name containing '{process_name}'...")
                for p in psutil.process_iter(['name']):
                    if process_name.lower() in p.info['name'].lower():
                        self.process = p
                        self.get_logger().info(f"Publishing CPU and memory usage for process {self.process.pid} ({self.process.name()})")
                        break
                time.sleep(2.0)
                
        self.timer = self.create_timer(0.5, self.timer_callback, clock = self.get_clock())

    def timer_callback(self):
        if self.process is None:
            cpu_percent = psutil.cpu_percent(interval=None)
            vm = psutil.virtual_memory()
            memory_percent = vm.percent
            Nthreads = 1
        else:
            Nthreads = self.process.num_threads()
            cpu_percent = self.process.cpu_percent(interval=None) 
            memory_percent = self.process.memory_percent()
        # self.get_logger().info(f"Published CPU load: {cpu_percent}%")
        if (cpu_percent is None) or (cpu_percent<=0.0) or (memory_percent is None) or (memory_percent<=0.0): return
        self.cpuload_pub.publish(std_msgs.msg.Float64(data=cpu_percent))
        self.cpuload_perthread_pub.publish(std_msgs.msg.Float64(data=cpu_percent / Nthreads))
        self.memory_pub.publish(std_msgs.msg.Float64(data=memory_percent))
        self.nthreads_pub.publish(std_msgs.msg.Int32(data=Nthreads))

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = PsutilPublisher()
    rclpy.spin(node)
if __name__ == '__main__':
    main()