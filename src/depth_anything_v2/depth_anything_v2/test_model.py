import cv2
import numpy as np
import sys
from transformers import pipeline
from PIL import Image
import requests
import torch

def main():
    # Choose GPU if available, otherwise CPU
    device = 0 if torch.cuda.is_available() else -1
    print("Using:", "cuda" if device == 0 else "cpu")

    # Load the HuggingFace depth estimation pipeline
    pipe = pipeline(
        task="depth-estimation",
        model="depth-anything/Depth-Anything-V2-Small-hf",
        device=device
    )

    # Load a test image (you can replace this URL or use a local file path)
    url = "http://images.cocodataset.org/val2017/000000039769.jpg"
    image = Image.open(requests.get(url, stream=True).raw).convert("RGB")

    # Run inference
    result = pipe(image)
    depth_pil = result["depth"]

    # Convert to OpenCV format (numpy array)
    depth = np.array(depth_pil)

    # Normalize for display (optional, enhances contrast)
    depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
    depth_uint8 = depth_norm.astype(np.uint8)

    # Apply a color map for better visualization
    depth_color = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_INFERNO)

    # Show result
    cv2.imshow("Original", np.array(image)[:, :, ::-1])  # RGB → BGR for OpenCV
    cv2.imshow("Depth Map", depth_color)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 
