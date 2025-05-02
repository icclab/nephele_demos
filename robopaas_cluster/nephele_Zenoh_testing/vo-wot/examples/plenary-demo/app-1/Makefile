all: app1

app1:
	cd ../../.. && make vo-wot
	docker build -t nepheleproject/app1 .

app1-minikube:
	cd ../../.. && make vo-wot-minikube
	@eval $$(minikube docker-env) ;\
		docker build -t nepheleproject/app1 .
