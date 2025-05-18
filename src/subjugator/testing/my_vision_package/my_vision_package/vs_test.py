import rclpy
import time
from PIL import Image
import numpy as np
from vision_stack import VisionStack, BinThresholdingLayer, CannyLayer

def main():
    rclpy.init()

    vs = VisionStack(
                layers=[
                    BinThresholdingLayer(150,250), # Converts image to grayscale if image is not grayscale and extracts pixels with values between 150 and 250.
                    #CannyLayer(50,100), # Simplified canny filter that uses cv2.Canny to pass a canny filter over an image with the low value (50) threshold for soft edge detection and the high value (100) for strong edges detection.
                ],
                unique_name="some_name_here" # TODO needs to be added
            )

    # Pass image through vision stack
    file_path = "/home/jh/mil2/src/subjugator/testing/my_vision_package/imgs/IMG_3063.jpg"

    # Open the image file
    img = Image.open(file_path)
     
    # Convert the image to RGB mode
    img = img.convert('RGB')
        
    # Convert the image to a NumPy array
    img_array = np.array(img)

    # Pass img_array through vision_stack
    vs.run(in_image = img_array, verbose = True) # TODO TYPO
    # With verbose, if ros is running then topics will be created for each layer to visualize processing.

    time.sleep(500)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
