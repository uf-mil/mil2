from subjugator_msgs.srv import Servo

from admission import adm

async def main():
    servo = Servo.Request()
    servo.angle = dropped
    await adm.fut(adm.dropper_srv.call_async(servo))

if __name__ == "__main__":
    adm.run(main())
