FROM python:3.11-slim
COPY . /vo-wot
WORKDIR /vo-wot
RUN pip install -U . --no-cache-dir
WORKDIR /
CMD ["vo-wot", "-f", "config.yaml", "-t", "td.json", "app.py"]
