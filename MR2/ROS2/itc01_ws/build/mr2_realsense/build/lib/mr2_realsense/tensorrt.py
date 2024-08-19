import os
from ultralytics import YOLO

# Load a YOLOv8n PyTorch model
model = YOLO(os.path.join(os.environ['HOME'], 'yolov8_rs', 'ball_yolov8t03.pt'))

# model = YOLO('ball_yolov8t03.pt')

# Export the model
results = model.export(format='engine') 


trt_model = YOLO('ball_yolov8t03.engine')

