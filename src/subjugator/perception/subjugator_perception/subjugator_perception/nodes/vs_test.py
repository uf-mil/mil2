#!/usr/bin/env python3
from vision_stack import VisionStack
import rclpy

def main():
    rclpy.init()
    print(VisionStack(layers=[]))
    print('Hi from subjugator_perception.')


if __name__ == '__main__':
    main()
