import os
import rclpy
from rclpy.node import Node
import threading
import sys, termios, tty
import yaml
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory
#from rclpy.parameter_client import AsyncParametersClient


def getch():
    """Read a single character from stdin without echo."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class ConfigSwitcher(Node):
    def __init__(self):
        super().__init__('config_switcher')
        # Async clients for Nav2 nodes
        self.planner_set_cli = self.create_client(SetParameters, 'planner_server/set_parameters')
        pkg_share = get_package_share_directory('turtlebot3_navigation2')
        self.config_dir = os.path.join(pkg_share, 'param')

        # Start keyboard listener thread
        threading.Thread(target=self._key_loop, daemon=True).start()
        #self.get_logger().info('ConfigSwitcher ready. Press "a" or "b".')


    def _key_loop(self):
        while rclpy.ok():
            self.get_logger().info('ConfigSwitcher ready. Press "a" or "b" and if you want STOP: "c".')
            ch = getch()
            if ch == 'a':
                filepath = os.path.join(self.config_dir, 'burger')
                self.get_logger().info(f'Loading {filepath}.yaml ...')
                self.load_use_astar('burger')
            elif ch == 'b':
                filepath = os.path.join(self.config_dir, 'burger2')
                self.get_logger().info(f'Loading {filepath}.yaml ...')
                self.load_use_astar('burger2')
            elif ch == 'c':
                self.get_logger().info(f'KeyBoard Interrupt....STOP')
                break


    def load_use_astar(self, which: str):
        """
        which: 'burger' 또는 'burger2'
        해당 YAML에서 planner_server → ros__parameters → GridBased → use_astar 값을 읽어와 설정
        """
        yaml_path = os.path.join(self.config_dir, f'{which}.yaml')
        try:
            with open(yaml_path, 'r') as f:
                raw = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'YAML 열기 실패: {e}')
            return

        use_astar_val = raw['planner_server']['ros__parameters']['GridBased']['use_astar']
        self.get_logger().warn(f'use_astar_val = {use_astar_val}')

        # Parameter 메시지 생성
        param = Parameter()
        param.name = 'GridBased.use_astar'
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value.bool_value = bool(use_astar_val)

        # 서비스 요청 준비
        req = SetParameters.Request()
        req.parameters = [param]

        # planner_server/set_parameters 서비스 호출
        future = self.planner_set_cli.call_async(req)
        future.add_done_callback(self._on_set_astar_done)


    def _on_set_astar_done(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'use_astar 서비스 호출 실패: {e}')
            return

        if res.results[0].successful:
            self.get_logger().info('use_astar 파라미터 설정 성공')
        else:
            self.get_logger().error(f'use_astar 설정 거부: {res}')    


def main(args=None):
    rclpy.init(args=args)
    node = ConfigSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()