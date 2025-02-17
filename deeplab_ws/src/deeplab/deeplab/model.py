import torch
import torch.nn as nn
from torchvision import transforms

import deeplab.network as network
from deeplab.cityscapes import Cityscapes

import cv2


class Model:
    def __init__(self):
        # Get device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Cityscapes dataset parameters
        self.decode_fn = Cityscapes.decode_target
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
            ),
        ])

        # Model backbone.
        # There is another possible choice: "deeplabv3plus_resnet101"
        # However, you will need to download the weight yourself.
        # Download url: https://github.com/VainF/DeepLabV3Plus-Pytorch#2-performance-on-cityscapes-19-classes-1024-x-2048
        self.backbone_name = "deeplabv3plus_mobilenet"

        # Download weight and load model
        self.download_weight()
        self.load_model()

    def download_weight(self):
        if self.backbone_name != "deeplabv3plus_mobilenet":
            return

        # Reference: https://github.com/VainF/DeepLabV3Plus-Pytorch#2-performance-on-cityscapes-19-classes-1024-x-2048
        url = "https://www.dropbox.com/s/753ojyvsh3vdjol/best_deeplabv3plus_mobilenet_cityscapes_os16.pth"
        output = "/home/ros2-agv-essentials/deeplab_ws/src/deeplab/deeplab/weights/best_deeplabv3plus_mobilenet_cityscapes_os16.pth"

        # Check if file exists
        from pathlib import Path

        is_file_exist = Path(output).is_file()

        # Download file if not exist
        if is_file_exist == False:
            try:
                import os
                os.system("wget " + url + " -O " + output)
            except Exception as error:
                print("Error: ", error)

    def load_model(self):
        # set True to speed up constant image size inference
        torch.backends.cudnn.benchmark = True

        # Weights path
        ckpt_path = (
            "/home/ros2-agv-essentials/deeplab_ws/src/deeplab/deeplab/weights/best_"
            + self.backbone_name
            + "_cityscapes_os16.pth"
        )

        # Load model
        self.model = network.modeling.__dict__[self.backbone_name](num_classes=19, output_stride=16)
        for m in self.model.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.momentum = 0.01

        # Load weight
        checkpoint = torch.load(ckpt_path, map_location=torch.device("cpu"))
        self.model.load_state_dict(checkpoint["model_state"])
        self.model = nn.DataParallel(self.model)
        self.model.to(self.device)
        del checkpoint

        self.model = self.model.eval()

    def inference(self, origin_img):
        with torch.no_grad():
            img = self.transform(origin_img).unsqueeze(0).to(self.device)
            pred = self.model(img).max(1)[1].cpu().numpy()[0]
            colorized_preds = self.decode_fn(pred).astype("uint8")
            result_img = cv2.addWeighted(colorized_preds, 0.5, origin_img, 0.5, 0)
        return result_img
