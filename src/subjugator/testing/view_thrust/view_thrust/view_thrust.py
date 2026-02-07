import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from subjugator_msgs.msg import ThrusterEfforts
from view_thrust.lut import newtons_from_duty

class ViewThrust(Node):
    def __init__(self):
        super().__init__("view_thrust")

        # requested thrusts in newtons
        self.last_wrench = Wrench()
        self.new_wrench = False
        self.pre_manager_wrench = self.create_subscription(
            Wrench,
            "cmd_wrench",
            self.wrench_cb,
            10,
        )

        # actual thrusts as a percent
        self.last_effort = ThrusterEfforts()
        self.new_effort = False
        self.post_manager_effort = self.create_subscription(
            ThrusterEfforts,
            "thruster_efforts",
            self.efforts_cb,
            10,
        )

        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def wrench_cb(self, msg: Wrench):
        self.last_wrench = msg
        if self.new_wrench:
            self.get_logger().error("dropped a wrench")
        self.new_wrench = True

    def efforts_cb(self, msg: ThrusterEfforts):
        self.last_effort = msg
        if self.new_effort:
            self.get_logger().error("dropped an effort")
        self.new_effort = True

    def timer_callback(self):
        # only debug if there is a new wrench and effort to report
        if not self.new_effort or not self.new_wrench:
            return

        # reset them
        self.new_wrench = False
        self.new_effort = False

        # print: wrench, effort, what the effort translates to on the purple board
        headers = (
            "wrench x",
            "wrench y",
            "wrench z",
            "wrench r",
            "wrench p",
            "wrench y",
        )
        rows = [
            (
                str(self.last_wrench.force.x),
                str(self.last_wrench.force.y),
                str(self.last_wrench.force.z),
                str(self.last_wrench.torque.x),
                str(self.last_wrench.torque.y),
                str(self.last_wrench.torque.z),
            ),
        ]
        self.get_logger().info("---------------------------------------------------")
        self.print_table(headers, rows)
        self.get_logger().info("\n")

        headers = (
            "unit",
            "FLH",
            "FLV",
            "FRH",
            "FRV",
            "BLH",
            "BLV",
            "BRH",
            "BRV"
        )
        rows = [
            (
                "duty%",
                str(round(self.last_effort.thrust_flh, 3)),
                str(round(self.last_effort.thrust_flv, 3)),
                str(round(self.last_effort.thrust_frh, 3)),
                str(round(self.last_effort.thrust_frv, 3)),
                str(round(self.last_effort.thrust_blh, 3)),
                str(round(self.last_effort.thrust_blv, 3)),
                str(round(self.last_effort.thrust_brh, 3)),
                str(round(self.last_effort.thrust_brv, 3)),
            ),
            (
                "Newtons",
                str(round(newtons_from_duty(self.last_effort.thrust_flh),2)),
                str(round(newtons_from_duty(self.last_effort.thrust_flv),2)),
                str(round(newtons_from_duty(self.last_effort.thrust_frh),2)),
                str(round(newtons_from_duty(self.last_effort.thrust_frv),2)),
                str(round(newtons_from_duty(self.last_effort.thrust_blh),2)),
                str(round(newtons_from_duty(self.last_effort.thrust_blv),2)),
                str(round(newtons_from_duty(self.last_effort.thrust_brh),2)),
                str(round(newtons_from_duty(self.last_effort.thrust_brv),2)),
            ),
        ]
        self.print_table(headers, rows)
        self.get_logger().info("---------------------------------------------------")

    def print_table(
        self,
        headers,
        rows,
    ):  # chat wrote this but it's formatting so i don't care
        """
        I added this tho:

        ### usage
        headers = ("Name", "Age", "City")
        rows = [
            ("Alice", 30, "New York"),
            ("Bob", 5, "Paris"),
            ("Catherine", 42, "San Francisco"),
        ]

        print_table(headers, rows)

        #### output of above
        | Name       | Age | City           |
        |------------|-----|----------------|
        | Alice      | 30  | New York       |
        | Bob        | 5   | Paris          |
        | Catherine  | 42  | San Francisco  |

        """
        table = [headers, *rows]
        widths = [max(len(str(row[i])) for row in table) for i in range(len(headers))]

        def print_row(row):
            self.get_logger().info(
                "| "
                + " | ".join(str(cell).ljust(widths[i]) for i, cell in enumerate(row))
                + " |",
            )

        print_row(headers)
        self.get_logger().info("|-" + "-|-".join("-" * w for w in widths) + "-|")
        for row in rows:
            print_row(row)
        self.get_logger().info("\n")


def main(args=None):
    rclpy.init(args=args)
    node = ViewThrust()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
