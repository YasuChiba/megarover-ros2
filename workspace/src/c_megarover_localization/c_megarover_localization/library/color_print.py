class COLOR_PRINT:
    def __init__(self, node):
        self.node = node

    def get_logger(self):
        return self.node.get_logger()

    def print_in_orange(self, msg):
        orange_start = "\033[38;5;208m"  # Orange color in 256-color mode.
        color_end = "\033[0m"
        self.get_logger().info(f"{orange_start}{msg}{color_end}")


    def print_in_blue(self, msg):
        blue_start = "\033[94m"
        color_end = "\033[0m"
        self.get_logger().info(f"{blue_start}{msg}{color_end}")

    def print_in_green(self, msg):
        green_start = "\033[92m"
        color_end = "\033[0m"
        self.get_logger().info(f"{green_start}{msg}{color_end}")

    def print_in_brown(self, msg):
        brown_start = "\033[38;5;130m"  # Brown color in 256-color mode.
        color_end = "\033[0m"
        self.get_logger().info(f"{brown_start}{msg}{color_end}")

    def print_in_purple(self, msg):
        purple_start = "\033[95m"  # Light magenta, which might look like purple in most terminals.
        color_end = "\033[0m"
        self.get_logger().info(f"{purple_start}{msg}{color_end}")

    def print_in_pink(self, msg):
        pink_start = "\033[38;5;213m"  # Alternative pink color in 256-color mode.
        color_end = "\033[0m"
        self.get_logger().info(f"{pink_start}{msg}{color_end}")

    def print_in_red(self, msg):
        red_start = "\033[31m"  # Red color.
        color_end = "\033[0m"
        self.get_logger().info(f"{red_start}{msg}{color_end}")

    def print_in_yellow(self, msg):
        yellow_start = "\033[33m"  # Yellow color.
        color_end = "\033[0m"
        self.get_logger().info(f"{yellow_start}{msg}{color_end}")