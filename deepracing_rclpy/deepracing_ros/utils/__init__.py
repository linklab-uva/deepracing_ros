import rclpy
import threading
class AsyncSpinner():
    def __init__(self, executor : rclpy.executors.Executor):
        self.executor : rclpy.executors.Executor = executor
        self.thread : threading.Thread = threading.Thread(target=self.executor.spin, daemon=True)
    def spin(self):
        self.thread.start()
    def addNode(self, node):
        self.executor.add_node(node)
    def shutdown(self, timeout_sec=0.0):
        self.executor.shutdown(timeout_sec = timeout_sec)