# YOLO Models

Place your YOLOv8 model weights here.

## Supported Formats

- `.pt` - PyTorch format (default)
- `.onnx` - ONNX format (for deployment)

## Pre-trained Models

Download from Ultralytics:

- `yolov8n.pt` - Nano (fastest)
- `yolov8s.pt` - Small (balanced) ✅ Recommended
- `yolov8m.pt` - Medium
- `yolov8l.pt` - Large
- `yolov8x.pt` - Extra Large

```bash
# Download model
pip install ultralytics
yolo export model=yolov8s.pt format=onnx
```

## Custom Model Training

For KRSBI competition, train custom model with:

- Class 0: Ball
- Class 1: Robot
- Class 2: Goalpost

### Training Dataset Structure

```
dataset/
├── train/
│   ├── images/
│   └── labels/
├── val/
│   ├── images/
│   └── labels/
└── data.yaml
```

### Training Command

```bash
yolo train model=yolov8s.pt data=data.yaml epochs=100 imgsz=416
```

## Model Performance

| Model   | mAP@50 | Inference (CPU) | Inference (GPU) |
| ------- | ------ | --------------- | --------------- |
| YOLOv8n | 37.3%  | ~45ms           | ~2ms            |
| YOLOv8s | 44.9%  | ~65ms           | ~3ms            |
| YOLOv8m | 50.2%  | ~150ms          | ~5ms            |

## Notes

- For real-time (30 fps), use YOLOv8s or smaller
- ONNX format is faster on CPU
- Consider TensorRT for NVIDIA GPUs
