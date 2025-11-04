import time
from typing import Any

import cv2
import torch


def midas_infer(img: cv2.typing.MatLike):

    # model_type = "MiDAS"
    # model_type = "DPT_Hybrid"
    model_type = "MiDaS_small"

    midas: Any = torch.hub.load("intel-isl/MiDaS", model_type)

    device = torch.device("mps") if torch.mps.is_available() else torch.device("cpu")
    midas.to(device)
    midas.eval()

    midas_transforms: Any = torch.hub.load("intel-isl/MiDaS", "transforms")

    if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
        transform = midas_transforms.dpt_transform
    else:
        transform = midas_transforms.small_transform

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    input_batch = transform(img).to(device)

    with torch.no_grad():
        start = time.time()
        prediction = midas(input_batch)
        took = time.time() - start

        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()
    print(f"took {took:.5f}s")

    output = prediction.cpu().numpy()

    return output
