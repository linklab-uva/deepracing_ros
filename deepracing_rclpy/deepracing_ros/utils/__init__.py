import rclpy, rclpy.executors as executors
import threading
class AsyncSpinner():
    def __init__(self, executor : executors.Executor, daemon=True):
        self.executor : executors.Executor = executor
        self.thread : threading.Thread = threading.Thread(target=self.executor.spin, daemon=daemon)
    def spin(self):
        self.thread.start()
    def add_node(self, node):
        self.executor.add_node(node)
    def shutdown(self, timeout_sec=0.0):
        self.executor.shutdown(timeout_sec = timeout_sec)