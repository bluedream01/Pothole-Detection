from flask import Flask, request, jsonify
from ultralytics import YOLO
from PIL import Image
import numpy as np
import time

app = Flask(__name__)

# ─────────────────────────────────────────────
# Load YOLO model
# ─────────────────────────────────────────────
model = YOLO("best (1).pt")

# 🔥 Warm-up (VERY IMPORTANT — removes 40–50s delay)
print("[INIT] Warming up model...")
dummy = np.zeros((320, 320, 3), dtype=np.uint8)
model(dummy)
print("[INIT] Model ready")


# ─────────────────────────────────────────────
# Routes
# ─────────────────────────────────────────────

@app.route("/", methods=["GET"])
def home():
    return "YOLO server is running"


@app.route("/upload", methods=["POST"])
def upload():
    start_time = time.time()

    try:
        # Check file
        if "image" not in request.files:
            return jsonify({"error": "No image provided"}), 400

        file = request.files["image"]

        # Read image
        image = Image.open(file.stream).convert("RGB")

        # 🔥 Resize for speed
        image = image.resize((320, 240))

        # Run YOLO (smaller imgsz = faster)
        results = model(image, conf=0.4, imgsz=320)

        detections = []
        for box in results[0].boxes:
            detections.append({
                "confidence": float(box.conf[0]),
                "bbox": box.xyxy[0].tolist()
            })

        # Message logic (Pi expects this)
        message = ""
        if detections:
            message = "Pothole has been detected."

        # Debug timing
        total_time = time.time() - start_time
        print(f"[INFO] Processed in {total_time:.2f}s | Detections: {len(detections)}")

        return jsonify({
            "detections": detections,
            "message": message
        })

    except Exception as e:
        print("[ERROR]", e)
        return jsonify({"error": str(e)}), 500


# ─────────────────────────────────────────────
# Run server
# ─────────────────────────────────────────────
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)