import rclpy
import threading
class AsyncSpinner():
    def __init__(self, executor):
        self.executor = executor
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
    def spin(self):
        self.thread.start()
    def addNode(self, node):
        self.executor.add_node(node)
    def shutdown(self):
        self.executor.shutdown()