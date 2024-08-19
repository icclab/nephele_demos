# Plenary meeting for the NEPHELE project in Lille
## Instructions to run the demo in minikube
For this demo a minikube installation of Kubernetes was used.
1. `cd` to the `mosquitto` directory. Afterwards, run the `mosquitto` broker using docker and leave running:
```bash
examples/plenary-demo/mosquitto$ docker run -it -p 1883:1883 -v ./mosquitto.conf:/mosquitto/config/mosquitto.conf -v ./passwd:/mosquitto/config/passwd_file eclipse-mosquitto:2
```
The above command runs a mosquitto MQTT broker in port 1883. The credentials necessary are `user` and `pass` as username and password respectively.

If the mosquitto clients (mosquitto_sub, mosquitto_pub) have been installed (available in the `mosquitto-clients` package for example) in order to check if the credentials have been propertly set, you can run the following command to subscribe to all topics:
```bash
mosquitto_sub -h localhost -u user -P pass -t '#' -v
```
and then publish to any topic:
```bash
mosquitto_pub -h localhost -u user -P pass -t 'test-topic' -m 'example-message'
```
2. Depending on whether a minikube installation is used, build the appropriate image. Building the application 1 image automatically builds the base wotpy image. The Makefile inside the app1 directory has two rules: `app1` and `app1-minikube`. The second rule builds the image inside the docker daemon of minikube.
Inside the `app-1` directory run:
```bash
examples/plenary-demo/app-1$ make app1-minikube
```

3. Mount the `vo-1`, `vo-2` and `cvo` directories if using minikube.
Run and leave the mounts running in separate terminals:
```bash
examples/plenary-demo$ cd vo-1
examples/plenary-demo/vo-1$ minikube mount $PWD:/mnt1
```
```bash
examples/plenary-demo$ cd vo-2
examples/plenary-demo$/vo-2$ minikube mount $PWD:/mnt2
```
```bash
examples/plenary-demo$ cd cvo
examples/plenary-demo/cvo$ minikube mount $PWD:/mnt
```

4. Run the device 1 script and leave it running. A convenience script using the cli is inside the scripts directory. The script assumes that the python package has been installed. In the root directory of the git repository run (preferably inside of a virtual environment):
```bash
(env)$ pip install vo-wot
```
Then in this directory run:
```bash
examples/plenary-demo(env)$ bash scripts/run-device-1.sh
```
This command will deploy the `device-1` and connect it to the MQTT broker. Leave the script running. Afterwards the `vo-1` can be deployed to consume it.

5. Deploy the VO1, VO2 and the cVO by running:
```bash
examples/plenary-demo$ bash scripts/deploy-vos.sh
```
This command will deploy the `vo-1` along with its influxdb, the `vo-2` and the `cvo` along with its influxdb. Running `kubectl get pods` should return these five pods.

6. Lastly, deploy the application 1 by running:
```bash
examples/plenary-demo$ kubectl apply -f app-1/deployment.yaml
```
This command will deploy the `app-1`. Running `kubectl get pods` should now return the previous five pods and the new app-1 pod.

7. The virtual objects and application 1 are exposed as NodePort Services. In the case that the minikube IP (can be fetched by running the command `minikube ip`) is `192.168.49.2` the endpoints of the application 1 can be accessed through these URLs.
```bash
http://192.168.49.2:30030/fire
http://192.168.49.2:30030/average_data
http://192.168.49.2:30030/forecast_data
```
To access the catalogue ports of the virtual objects where the Thing Descriptions are served, access these URLs:
```bash
http://192.168.49.2:30000/vo1
http://192.168.49.2:30013/vo2
http://192.168.49.2:30020/cvo
```

8. To send a smoke signal and by extension trigger the fire detection, run the device 2 script. Again this assumes that the python package is installed. So in the root directory of the git repository run (preferably inside of a virtual environment):
```bash
(env)$ pip install vo-wot
```
And then in this directory run:
```bash
examples/plenary-demo(env)$ bash scripts/run-device-2.sh
```
This command sends the smoke signal from the `device-2` to the `vo-2`. This change is then picked up by the `cvo` which checks the temperature and in combination with the smoke, determines if there is a fire. The presence of fire can be checked by accessing the endpoint `http://192.168.49.2:30030/fire` of the `app-1`.

## Instructions to run the demo in real k8s cluster
For this demo a k8s cluster is needed.  The device 1, device 2 and the mqtt broker run locally, while the vo1 runs on the k8s cluster. Be sure that your local IP is accesible from the k8s cluster and viceversa.
1. Install the mosquitto MQTT broker (OS dependent). Then make sure the configuration file (for example the one in `/etc/mosquitto/mosquitto.conf`) has the following lines:
```
listener 1883
allow_anonymous false
```


(Optional) If a username/password needs to be specified (for example "user" as username and "pass" as password) follow the official instructions on creating a password file and then specify the password file's path in the configuration file by adding a line similar to:
```
password_file /etc/mosquitto/passwd
```
To check if the credentials have been propertly set, you can run the following command to subscribe to all topics:
```bash
mosquitto_sub -h localhost -u <your_username> -P <your_password> -t '#' -v
```
and then publish to any topic:
```bash
mosquitto_pub -h localhost -u <your_username> -P <your_password> -t 'test-topic' -m 'example-message'
```
If the username and password are set, make sure to also set the same credentials in the:
- `examples/plenary-demo/vo-1/vo-1.yaml`:
```yaml
      securitySB:
          securitySBMQTT:
            security_scheme: basic
            username: <your_username>
            password: <your_password>
```
- `examples/plenary-demo/device-1.yaml`:
```yaml
    securityNB:
      security_scheme: basic
      username: <your_username>
      password: <your_password>
```

2. The appropriae wotpy image is already build and hosted at the dockerhub repository https://hub.docker.com/u/netmode. So you can skip this step. However, If you want to store the wotpy image at your own dockerhub repository, you can update the Makefile and run inside the `nephele_virtual_object_wot` directory:
```bash
make vo-wot
```

3. At this step you will instantiate the device 1. Before running the device 1 script you should update the `device_1.yaml` file. More in detail update the fields of `hostname` and `brokerIP` with you local IP. After that, run the device 1 script locally and leave it running. A convenience script using the cli is inside the scripts directory. The script assumes that the python package has been installed. In the root directory of the git repository run (preferably inside of a virtual environment):
```bash
pip install vo-wot
```
Then in this directory run:
```bash
bash scripts/run-device-1.sh
```

4. Deploy the VO 1:curr_device_status
Firstly update the vo-1/vo-1.py with your local IP adress. More in detail update the `url` and the `curr_device_status` variables.
You should also update the vo-1/deployment.yaml.
```
Replace the wotpy image with the one that comes from the docker repository
-        image: nepheleproject/vo-wot
-        imagePullPolicy: Never
+        image: nepheleproject/vo-wot
+        imagePullPolicy: IfNotPresent
```
Similarlly, update the wot-script hostpath to the point the vo-1 folder.
```
         hostPath:
-          path: /mnt1/
+          path: /home/ubuntu/nephele_virtual_object_wot/examples/plenary-demo/vo-1/
       - name: wot-config
         hostPath:
-          path: /mnt1/
+          path: /home/ubuntu/nephele_virtual_object_wot/examples/plenary-demo/vo-1/
```
Then  deploy the VO 1 by running:
```bash
bash scripts/deploy-vo1-2.sh
```

5. Check out the status of VO-1
```
kubectl get services
kubectl get pods
kubectl describe pods vo1-59d6f4cfb8-d6rnr
```
if you want to delete the vo1 deployment:
```
kubectl delete -f /vo-2/deployment.yaml
```
You can acess the vo1 within the cluster with :
```
curl http://localhost:30000/vo1
```
Or enable port forwarding and acces it via your local machine
```
ssh -L 30000:localhost:30000 k8sHostmachineUser@k8sHostmachineIP
```

