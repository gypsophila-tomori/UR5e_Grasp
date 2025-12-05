import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama
import json

class LLMControlNode(Node):
    """
    接收自然语言指令，调用Ollama LLM生成结构化任务计划，
    并将计划以JSON字符串的形式发布出去。
    """
    def __init__(self):
        super().__init__('llm_control_node')

        # --- 参数定义 ---
        # 声明可配置的参数，增加灵活性
        self.declare_parameter('model_name', 'qwen2.5:3b')
        self.declare_parameter('input_topic', '/llm_command')
        self.declare_parameter('output_topic', '/llm_plan')
        
        # 从参数服务器获取值
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # --- ROS2 通信设置 ---
        # 创建订阅者，接收来自CLI或UI的自然语言指令
        self.command_subscription = self.create_subscription(
            String,
            input_topic,
            self.command_callback,
            10)
        
        # 创建发布者，将LLM生成的JSON计划发布出去
        self.plan_publisher = self.create_publisher(String, output_topic, 10)

        self.get_logger().info(f"LLM Control Node initialized. Using model: '{self.model_name}'.")
        self.get_logger().info(f"Listening for commands on: '{input_topic}'")
        self.get_logger().info(f"Publishing plans to: '{output_topic}'")

    def command_callback(self, msg):
        """当接收到新指令时被调用的回调函数"""
        command_text = msg.data
        self.get_logger().info(f'Received command: "{command_text}"')

        # 调用Ollama生成计划
        action_plan_json = self.generate_plan(command_text)

        # 如果成功生成计划，则发布
        if action_plan_json:
            plan_msg = String()
            plan_msg.data = action_plan_json
            self.plan_publisher.publish(plan_msg)
            self.get_logger().info(f"Successfully generated and published plan.")
            self.get_logger().info(f"Plan details: {action_plan_json}")
        else:
            self.get_logger().error("Failed to generate a plan from the command.")

# 在 llm_control_node.py 中

    def generate_plan(self, command_text: str) -> str | None:
        """
        构建Prompt并调用Ollama LLM，返回一个JSON格式的计划字符串。
        这个版本更健壮，可以处理模型返回的多余文本。
        """
        # --- 提示工程 (Prompt Engineering) - 和之前一样 ---
        prompt = f"""
        你是一个机器人任务规划助手。你的任务是将人类的自然语言指令，转换成一个由机器人动作组成的JSON对象数组。

        # 可用动作函数:
        1. `pick(object_name)`: 从场景中抓取一个指定的物体。参数 `object_name` 必须是字符串。
        2. `place(position_name)`: 将手中当前持有的物体，放置到一个预定义的位置。参数 `position_name` 必须是字符串。

        # 场景上下文:
        - 当前场景中可供抓取的物体有: ["cube1", "cube2", "cube3"]
        - 当前场景中可供放置的区域有: ["red_bin", "green_bin", "blue_bin", "center_table"]

        # 任务:
        根据以下这条人类指令，生成一个JSON格式的动作序列。

        # 要求:
        - 你的回答必须是且仅是一个格式正确的JSON代码块。
        - 不要包含任何解释、注释或其他多余的文字。
        - 如果指令无法理解或无法执行，请返回一个空的JSON数组 `[]`。

        # 人类指令:
        "{command_text}"
        """

        try:
            self.get_logger().info("Sending request to Ollama...")
            response = ollama.chat(
                model=self.model_name,
                messages=[{'role': 'user', 'content': prompt}],
                format='json'
            )
            
            response_text = response['message']['content']
            self.get_logger().info(f"RAW OLLAMA RESPONSE:\n---\n{response_text}\n---") # 打印原始输出，非常有助于调试！

            # --- 新增的健壮性代码 ---
            # 找到JSON内容的开始和结束位置
            # 有的JSON以'{'开始(单个对象), 有的以'['开始(数组)
            start_bracket = min(response_text.find('{'), response_text.find('['))
            if start_bracket == -1: # 如果找不到 '{' 或 '['
                self.get_logger().error("No JSON object or array found in the response.")
                return None
            
            # 找最后一个 '}' 或 ']'
            end_bracket = max(response_text.rfind('}'), response_text.rfind(']'))
            if end_bracket == -1:
                 self.get_logger().error("Could not find the end of the JSON content.")
                 return None

            # 提取纯净的JSON字符串
            json_str = response_text[start_bracket : end_bracket+1]
            
            # 尝试解析提取出的字符串，确保它是有效的
            plan_dict = json.loads(json_str)
            
            # 成功解析后，返回这个纯净的JSON字符串
            return json.dumps(plan_dict)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON from Ollama's response: {e}")
            self.get_logger().error(f"Content that failed parsing: {response_text}")
            return None
        except Exception as e:
            self.get_logger().error(f"An error occurred while communicating with Ollama: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    llm_control_node = LLMControlNode()
    rclpy.spin(llm_control_node)
    # 节点关闭时销毁
    llm_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
