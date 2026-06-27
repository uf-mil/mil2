import admission as adm
from geometry_msgs.msg import Pose

async def estimate_bins():
    async for yolo, odom in adm.Join(adm.yolo_sub, adm.odom_sub):
        if yolo:
            for det in yolo.detections:
                print(det.bbox.center.position.x, det.bbox.center.position.y)
            print("-" * 40)
        elif odom:
            pass

if __name__ == "__main__":
    adm.run(estimate_bins())
