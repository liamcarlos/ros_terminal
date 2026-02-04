import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from rclpy.executors import MultiThreadedExecutor

# --- IPython Imports mirroring mainloop.py ---
from IPython.terminal.embed import InteractiveShellEmbed
from IPython.terminal.prompts import Prompts, Token
from traitlets.config.loader import Config

# 1. Define Custom Prompts like in mainloop.py 
class ROSPrompt(Prompts):
    def in_prompt_tokens(self):
        return [(Token.Prompt, 'ros2> ')]

    def out_prompt_tokens(self):
        return [(Token.OutPrompt, 'Result: ')]

class InteractiveRosNode(Node):
    def __init__(self):
        super().__init__('interactive_node')
        self.pub = self.create_publisher(PoseArray, 'interactive_poses', 10)
        self.string_pub = self.create_publisher(String, 'interactive_strings', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.string = String()
        self.string.data = "Hello from Interactive ROS Node"
        self.get_logger().info("ROS 2 Interactive Node Initialized")

    def send_pattern(self, count=5, spacing=1.0):
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = "world"
        for i in range(count):
            p = Pose()
            p.position.x = float(i * spacing)
            pa.poses.append(p)
        self.pub.publish(pa)
        print(f"Sent {count} poses.")

    def timer_callback(self):
        self.string_pub.publish(self.string)

def ros_terminal_processor(node, local_vars):
    # 2. Set Configuration Options like in mainloop.py 
    cfg = Config()
    cfg.TerminalInteractiveShell.prompts_class = ROSPrompt  # Fixes the TraitError 
    cfg.TerminalInteractiveShell.term_title = True
    cfg.TerminalInteractiveShell.term_title_format = "ROS 2 Interactive Shell"

    # 3. Instantiate the shell as done in the source 
    ipshell = InteractiveShellEmbed.instance(config=cfg)
    
    print("\n--- ROS 2 Robust Terminal ---")
    print("Available: 'ros' (the node), 'rclpy'")
    
    # 4. Embed with local variables 
    ipshell(local_ns=local_vars)

def main():
    if not rclpy.ok():
        rclpy.init()
    
    ros_node = InteractiveRosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    # 5. Background Threading like in dataguzzler_python.py [cite: 2]
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    local_env = {
        'ros': ros_node,
        'rclpy': rclpy,
        'Pose': Pose,
        'PoseArray': PoseArray,
    }

    try:
        ros_terminal_processor(ros_node, local_env)
    finally:
        # 6. Proper shutdown [cite: 1, 2]
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
