# Plenary meeting for the NEPHELE project in Lille
## Instructions to run the demo
For this demo an installation of Kubernetes (either a real cluster or a minikube install) was used.
1. Install the mosquitto MQTT broker locally (OS dependent). Then make sure the configuration file (for example the one in `/etc/mosquitto/mosquitto.conf`) has the following lines:
```
listener 1883
allow_anonymous false
```


If a username/password needs to be specified (for example "user" as username and "pass" as password) follow the official instructions on creating a password file and then specify the password file's path in the configuration file by adding a line similar to:
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
- `examples/plenary-demo-helmchart-helmchart/charts/vo1/scripts/config.yaml`:
```yaml
      securitySB:
          securitySBMQTT:
            security_scheme: basic
            username: <your_username>
            password: <your_password>
```
- `examples/plenary-demo-helmchart-helmchart/device_1.yaml`:
```yaml
    securityNB:
      security_scheme: basic
      username: <your_username>
      password: <your_password>
```

2. Run the device 1 script and leave it running. A convenience script using the cli is inside the scripts directory. The script assumes that the python package has been installed. In the root directory of the git repository run (preferably inside of a virtual environment):
```bash
(env)$ pip install vo-wot
```
Then in this directory run:
```bash
examples/plenary-demo-helmchart(env)$ bash scripts/run-device-1.sh
```
This command will deploy the `device-1` and connect it to the MQTT broker. Leave the script running. Afterwards the `vo-1` can be deployed to consume it.

3. Cd into the helmchart directory, change the `values.yaml` and deploy the chart.
```bash
examples/plenary-demo-helmchart$ cd helmchart
examples/plenary-demo-helmchart/helmchart$ vim values.yaml
```
The `values.yaml` has two keys inside for now. The `deviceIP` and the `brokerURL`. For minikube these are set to:
```yaml
deviceURL: "http://192.168.49.1:9090/device1"
brokerIP: "mqtt://192.168.49.1:1883"
```
If the MQTT broker is deployed at the `<broker_url>` URL and the device1 is deployed on an interface with IP `<device_IP>` then change the `values.yaml` as such:
```yaml
deviceURL: "http://<device_IP>:9090/device1"
brokerIP: "mqtt://<broker_url>:1883"
```
Lastly, deploy the helmchart:
```bash
examples/plenary-demo-helmchart/helmchart$ helm install graph .
```
This will install the entire graph containing two Virtual Objects (vo1, vo2), one composite Virtual Object (cvo), two InfluxDB database pods (one for the vo1 and one one for the cvo) and finally the app1 component.


4. The virtual objects and application 1 are exposed as NodePort Services. In the case that the minikube IP (can be fetched by running the command `minikube ip`) is `192.168.49.2` the endpoints of the application 1 can be accessed through these URLs.
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
If deployed on a real cluster substitute the URL `192.168.49.2` with the Kubernetes Node URL where the desired pod is deployed.

5. To send a smoke signal and by extension trigger the fire detection, run the device 2 script. Again this assumes that the python package is installed. So in the root directory of the git repository run (preferably inside of a virtual environment):
```bash
$ pip install vo-wot
```
And then in this directory run:
```bash
examples/plenary-demo-helmchart(env)$ bash scripts/run-device-2.sh
```
This command sends the smoke signal from the `device-2` to the `vo-2`. This change is then picked up by the `cvo` which checks the temperature and in combination with the smoke, determines if there is a fire. The presence of fire can be checked by accessing the endpoint `http://192.168.49.2:30030/fire` of the `app-1`.
