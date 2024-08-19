all: vo-wot

vo-wot:
	docker build -t nepheleproject/vo-wot .

vo-wot-minikube:
	@eval $$(minikube docker-env) ;\
		docker build -t nepheleproject/vo-wot .

.PHONY: all vo-wot
