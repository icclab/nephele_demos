import base64
import io
import threading

import requests
from PIL import Image


def forward_request(buffer):
    dns_name = 'noise-reduction.default.svc.clusterset.local'
    url = f'http://{dns_name}:8080/'
    files = {
        'image': buffer.getvalue(),
    }

    requests.post(url, files=files)


async def detectImage_handler(params):
    input = params['input']
    image = input['image']

    image_bytes = base64.b64decode(image.encode('ascii'))
    img = Image.open(io.BytesIO(image_bytes))

    buffer = io.BytesIO()
    img.save(buffer, format='jpeg', optimize=True, quality=50)

    thread = threading.Thread(
        target=forward_request,
        args=(buffer,)
    )
    thread.start()

    return 'Request forwarded'
