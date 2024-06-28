FROM python:3.10-slim

WORKDIR /home/app

RUN apt-get update \
    && apt-get install ffmpeg libsm6 libxext6 -y
RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu --no-cache-dir
COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt --no-cache-dir
COPY app.py app.py

ENV PROMETHEUS_MULTIPROC_DIR /tmp
ENV prometheus_multiproc_dir /tmp

CMD exec gunicorn --bind 0.0.0.0:8080 --workers 2 --threads 2 --timeout 0 --log-level=debug app:app
