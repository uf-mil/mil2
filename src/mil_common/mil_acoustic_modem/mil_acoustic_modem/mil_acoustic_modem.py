# This node will run on the boat and sub

# params:
# - serial port

import rclpy
from mil_msgs.msg import PipelineSurveyReport
from rclpy.node import Node


class PipelineSurveyReportPublisher(Node):
    def __init__(self):
        super().__init__("acoustic_publisher")
        self.publisher = self.create_publisher(
            PipelineSurveyReport,
            "pipeline_survey_report_received",
            5,
        )


def main(args=None):
    rclpy.init(args=args)

    pipeline_survey_report_publisher = PipelineSurveyReportPublisher()

    rclpy.spin(pipeline_survey_report_publisher)

    pipeline_survey_report_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
