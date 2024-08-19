import logging
import threading

import cv2 as cv
import numpy as np
import prometheus_flask_exporter
import requests
from flask import Flask, request
from prometheus_flask_exporter.multiprocess import GunicornPrometheusMetrics

app = Flask(__name__)
metrics = GunicornPrometheusMetrics(app)
metrics = prometheus_flask_exporter.PrometheusMetrics(app)

if __name__ != "__main__":
    gunicorn_logger = logging.getLogger('gunicorn.error')
    app.logger.handlers = gunicorn_logger.handlers
    app.logger.setLevel(gunicorn_logger.level)


def forward_request(image_bytes):
    dns_name = 'image-detection.default.svc.clusterset.local'
    url = f'http://{dns_name}:8080/'
    files = {
        'image': image_bytes
    }
    requests.post(url, files=files)


@app.route('/', methods=['POST'])
def noise_reduction():
    if 'image' not in request.files:
        return {'error': 'Missing input image'}
    image_file = request.files['image']
    image_bytes = image_file.read()

    np_arr = np.frombuffer(image_bytes, dtype=np.uint8)
    img = cv.imdecode(np_arr, cv.IMREAD_COLOR)
    res = cv.fastNlMeansDenoisingColored(img, None, 3, 3, 7, 21)
    image_bytes = cv.imencode('.jpg', res)[1].tobytes()

    thread = threading.Thread(
        target=forward_request,
        args=(image_bytes,)
    )
    thread.start()

    return 'Request received', 202


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=8080)
