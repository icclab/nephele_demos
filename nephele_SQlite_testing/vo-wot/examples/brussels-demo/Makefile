HDAR := nephele-platform.netmode.ece.ntua.gr
PROJECT := brussels-demo

default: help

all help: ## Show this help.
	@grep -F -h "##" $(MAKEFILE_LIST) | grep -F -v grep -F | sed -e 's/\\$$//' | sed -e 's/##//'

build-images: ## cd into the src/ directory, build all docker images and push them to dockerhub
	echo "Creating docker images and pushing to dockerhub..."
	$(MAKE) -C src/

registry-login: ## Login to the HDAR registry
	echo "Logging in to HDAR located at ${HDAR}..."
	hdarctl login ${HDAR}

package-artifacts: ## Package artifacts
	echo "Packaging artifacts..."
	hdarctl package tar vo/
	hdarctl package tar hdag/
	helm package image-detection/
	helm package noise-reduction/

push-artifacts: registry-login package-artifacts ## Push packaged artifacts to the HDAR them to the HDAR
	echo "Pushing artifacts..."
	hdarctl push image-compression-vo-0.1.0.tar.gz  ${HDAR}/${PROJECT}
	hdarctl push image-detection-graph-1.0.0.tar.gz ${HDAR}/${PROJECT}
	hdarctl push image-detection-0.1.0.tgz ${HDAR}/${PROJECT}
	hdarctl push noise-reduction-0.1.0.tgz ${HDAR}/${PROJECT}

.PHONY: all build-images registry-login package-artifacts push-artifacts help
