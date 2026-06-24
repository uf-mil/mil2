import admission as adm
from geometry_msgs.msg import Pose

async def estimate_bins():
    while yolo := await adm.yolo_sub:
        for det in yolo.detections:
            print(det.bbox.center.position.x, det.bbox.center.position.y)
        print("-" * 40)

if __name__ == "__main__":
    adm.run(estimate_bins())
