REGISTRY_PREFIX := nepheleproject
IMAGE_NAME := noise-reduction

all: build push

build:
	docker build -t "${REGISTRY_PREFIX}/${IMAGE_NAME}" .

push:
	docker push "${REGISTRY_PREFIX}/${IMAGE_NAME}"
