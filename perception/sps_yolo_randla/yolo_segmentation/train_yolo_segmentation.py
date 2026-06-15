from ultralytics import YOLO

# model = YOLO("pretrained/yolo12s.pt")  # Load a pretrained YOLOv12 model
# model = YOLO("pretrained/yolo11m-seg.pt")  # Load a pretrained YOLOv12 model
model = YOLO("ckpts/yolo12_seg.pt")

# ckpt_path = "runs/detect/train/weights/best.pt"
# model = YOLO(ckpt_path)
# Train the model on a custom dataset

# The following parameters can be adjusted based on your dataset and training requirements:
# data (str): Path to dataset configuration file.
# epochs (int): Number of training epochs.
# batch (int): Batch size for training.
# imgsz (int): Input image size.
# device (str): Device to run training on (e.g., 'cuda', 'cpu').
# workers (int): Number of worker threads for data loading.
# optimizer (str): Optimizer to use for training.
# lr0 (float): Initial learning rate.
# patience (int): Epochs to wait for no observable improvement for early stopping of training.
results = model.train(data="sample_part_segmentation/data.yaml",
                      epochs=100,
                      imgsz=640,
                      batch=16,
                      device="0",
                      workers=4,
                      optimizer="SGD",
                      lr0=0.001,
                      patience=30)  # Early stopping after 30 epochs without improvement
print(results)  # Print training results

# results = model.predict("sample_part_image_head_rgb.jpg")
# results[0].show()  # Display the results of inference
