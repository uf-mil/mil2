#!/usr/bin/env python3
"""Runs the coin_flip wall-orientation classifier and publishes its prediction.

coin_flip.pt is NOT a YOLO model. It is a ResNet-18 image
classifier whose checkpoint stores {model_state, arch, classes, img_size, ...}.
The classes are ["Right", "Left", "Front", "Wall", "Other"] and describe where
the wall is relative to the sub. This node subscribes to the front camera, runs
the classifier, and publishes the lower-cased winning class as a std_msgs/String
on /coin_flip/direction so the mission planner can react to it.

If the max-class probability is below `min_confidence`, "other" is published so
downstream consumers treat the frame as undecided.
"""

import os

import cv2
import numpy as np
import rclpy
import torch
import torch.nn as nn
import torch.nn.functional as F
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


# --- Model definition (matches the keys saved in coin_flip.pt) ----------------
class BasicBlock(nn.Module):
    """Standard ResNet basic block."""

    def __init__(self, inp, out, stride=1, downsample=None):
        super().__init__()
        self.conv1 = nn.Conv2d(inp, out, 3, stride, 1, bias=False)
        self.bn1 = nn.BatchNorm2d(out)
        self.conv2 = nn.Conv2d(out, out, 3, 1, 1, bias=False)
        self.bn2 = nn.BatchNorm2d(out)
        self.downsample = downsample
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        identity = x
        out = self.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        if self.downsample is not None:
            identity = self.downsample(x)
        return self.relu(out + identity)


class ResNetScratch(nn.Module):
    """ResNet-18 topology with a `stem` (conv+bn) instead of conv1/bn1."""

    def __init__(self, num_classes=5, dropout=0.4):
        super().__init__()
        self.stem = nn.Sequential(
            nn.Conv2d(3, 64, 7, 2, 3, bias=False),
            nn.BatchNorm2d(64),
        )
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(3, 2, 1)
        self.inp = 64
        self.layer1 = self._make_layer(64, 2, 1)
        self.layer2 = self._make_layer(128, 2, 2)
        self.layer3 = self._make_layer(256, 2, 2)
        self.layer4 = self._make_layer(512, 2, 2)
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
        self.dropout = nn.Dropout(dropout)
        self.fc = nn.Linear(512, num_classes)

    def _make_layer(self, out, blocks, stride):
        downsample = None
        if stride != 1 or self.inp != out:
            downsample = nn.Sequential(
                nn.Conv2d(self.inp, out, 1, stride, bias=False),
                nn.BatchNorm2d(out),
            )
        layers = [BasicBlock(self.inp, out, stride, downsample)]
        self.inp = out
        for _ in range(1, blocks):
            layers.append(BasicBlock(out, out))
        return nn.Sequential(*layers)

    def forward(self, x):
        x = self.maxpool(self.relu(self.stem(x)))
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)
        x = torch.flatten(self.avgpool(x), 1)
        return self.fc(self.dropout(x))


class CoinFlipNode(Node):
    def __init__(self):
        super().__init__("coin_flip_node")

        default_model = os.path.join(
            get_package_share_directory("subjugator_vision"),
            "models",
            "coin_flip.pt",
        )

        self.declare_parameter("image_topic", "/front_cam/image_raw")
        self.declare_parameter("output_topic", "/coin_flip/direction")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("device", "cpu")
        # ImageNet normalization by default; override if the model was trained
        # with different preprocessing.
        self.declare_parameter("mean", [0.485, 0.456, 0.406])
        self.declare_parameter("std", [0.229, 0.224, 0.225])

        self.min_conf = float(self.get_parameter("min_confidence").value)
        self.device = torch.device(self.get_parameter("device").value)
        self.mean = np.array(self.get_parameter("mean").value, dtype=np.float32)
        self.std = np.array(self.get_parameter("std").value, dtype=np.float32)

        model_path = self.get_parameter("model_path").value
        ckpt = torch.load(model_path, map_location="cpu", weights_only=False)
        self.classes = list(ckpt["classes"])
        self.img_size = int(ckpt.get("img_size", 224))

        self.model = ResNetScratch(
            num_classes=len(self.classes),
            dropout=float(ckpt.get("dropout", 0.0)),
        )
        self.model.load_state_dict(ckpt["model_state"])
        self.model.to(self.device).eval()

        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            String,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.on_image,
            10,
        )
        self.get_logger().info(
            f"coin_flip_node up: model={model_path} classes={self.classes} "
            f"img_size={self.img_size} min_conf={self.min_conf}",
        )

    def preprocess(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.img_size, self.img_size))
        arr = rgb.astype(np.float32) / 255.0
        arr = (arr - self.mean) / self.std
        # HWC -> CHW -> NCHW
        tensor = torch.from_numpy(arr.transpose(2, 0, 1)).unsqueeze(0)
        return tensor.to(self.device)

    def on_image(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with torch.no_grad():
            logits = self.model(self.preprocess(bgr))
            probs = F.softmax(logits, dim=1)[0]
            conf, idx = torch.max(probs, dim=0)

        label = self.classes[int(idx)].lower()
        conf = float(conf)
        if conf < self.min_conf:
            label = "other"

        self.pub.publish(String(data=label))
        self.get_logger().debug(f"coin_flip: {label} ({conf:.2f})")


def main():
    rclpy.init()
    node = CoinFlipNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
