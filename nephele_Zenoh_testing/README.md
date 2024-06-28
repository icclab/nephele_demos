This repository contains the deployment files and container image creation files for the robotic demo.


Use the following command to deploy on the kubernetes cluster:

``
kubectl apply -k k8s_deployment -n <namespace>
``

You will deploy:

- Foxglove studio container with a default layout (using a configmap)
- Foxglove bridge and Zenoh bridge container colocated with Foxglove studio container for easier communication
- Zenoh Router
- MQTT broker
- Simple Flask webapp
- An nginx webserver to host the tb2 Thing description

After the deployment you can reach the Foxglove Studio (using Chrome webbrowser) at https://foxglove-demo.robopaas.dev/ and open a websocket connection to the bridge to reach the ROS world towards this ingress wss://bridges.robopaas.dev 
Any ROS topic published to the same Zenoh Router can be shown in the Foxglove dashboard.

The MQTT broker is going to used to expose data from ROS to the VO and send actions from the VO (starting from the webapp) to the tb2

The flask webapp GUI can be reached over a K8s ingress at flask-gui.robopaas.dev

N.B. You might need to change the NodePorts used!
