# Smaller demo with only the VO1 and app1 components
## Instructions to run the demo in minikube
For this demo a minikube installation of Kubernetes was used.
1. Depending on whether a minikube installation is used, build the appropriate image. Building the application 1 image automatically builds the base wotpy image. The Makefile inside the app1 directory has two rules: `app1` and `app1-minikube`. The second rule builds the image inside the docker daemon of minikube.
Run inside the `app-1` directory:
```bash
examples/plenary-demo/app-1$ make app1-minikube
```

2. Mount the `vo-1` and `app-1` directories to minikube. Run and leave the mounts running.
For vo-1:
```bash
examples/plenary-demo$ cd vo-1
examples/plenary-demo/vo-1$ minikube mount $PWD:/mnt1
```
For app1:
```bash
examples/plenary-demo$ cd app-1
examples/plenary-demo/app-1$ minikube mount $PWD:/mnt-app1
```

3. Deploy the vo-1 and app-1 by running:
```bash
examples/plenary-demo$ bash scripts/deploy-all.sh
```
This command will deploy the `vo-1` and `app-1`. Running `kubectl get pods` should return 2 pods: `app-1` and `vo1`. `app-1` has 2 containers (the app itself and the Holder component) while `vo1` has 4 containers (the VO , the Holder component, the Verifier component and the Proxy component).


4. The virtual object and application 1 are exposed as NodePort Services. In the case that the minikube IP (can be fetched by running the command `minikube ip`) is `192.168.49.2` the endpoints of the application 1 can be accessed through these URLs.
```bash
http://192.168.49.2:30030/temperature
http://192.168.49.2:30030/humidity
```
To access the catalogue ports of the virtual object where the Thing Descriptions is served, access this URL:
```bash
http://192.168.49.2:30000/vo1
```
