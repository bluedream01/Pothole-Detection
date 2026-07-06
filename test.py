from ultralytics import YOLO
YOLO("best (1).pt").export(format="ncnn")   # → creates best_ncnn_model/