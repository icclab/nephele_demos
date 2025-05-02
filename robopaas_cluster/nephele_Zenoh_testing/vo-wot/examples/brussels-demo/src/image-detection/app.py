import io
import logging

import prometheus_flask_exporter
from flask import Flask, request
from PIL import Image
from prometheus_flask_exporter.multiprocess import GunicornPrometheusMetrics
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
app = Flask(__name__)
metrics = GunicornPrometheusMetrics(app)
metrics = prometheus_flask_exporter.PrometheusMetrics(app)

if __name__ != "__main__":
    gunicorn_logger = logging.getLogger('gunicorn.error')
    app.logger.handlers = gunicorn_logger.handlers
    app.logger.setLevel(gunicorn_logger.level)


@app.route('/', methods=['POST'])
def detect_image():
    if 'image' not in request.files:
        return {'error': 'Missing input image'}
    image_file = request.files['image']
    image_bytes = image_file.read()

    img = Image.open(io.BytesIO(image_bytes))
    results = model(img)

    return 'Request received', 202


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=8080)
