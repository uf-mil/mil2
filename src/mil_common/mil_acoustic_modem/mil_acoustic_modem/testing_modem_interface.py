from google.protobuf import any_pb2

from mil_acoustic_modem.modem_interface import ModemInterface
from mil_acoustic_modem.protobuf import pipeline_survey_report_pb2


class TestingModemInterface(ModemInterface):
    def read_im(self):
        any_message = any_pb2.Any()
        pipeline_survey_report_message = (
            pipeline_survey_report_pb2.PipelineSurveyReport()
        )

        pipeline_survey_report_message.active_buoy_position_x = 0
        pipeline_survey_report_message.active_buoy_position_y = 0
        pipeline_survey_report_message.segments.append(0)

        any_message.Pack(pipeline_survey_report_message)

        protobuf_string = any_message.SerializeToString()

        return str.encode("RECVIM,1,1,1,ack,1,1,1,1,") + protobuf_string

    def send_im(self, message):
        print(f"TestingModemInterface: Sending message '{message}'")
        pass
