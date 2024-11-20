This repository contains the deployment files and container image creation files for the robotic demo.


Use the following command to deploy on the kubernetes cluster:

``
kubectl apply -k k8s_deployment -n <namespace>
``

You will deploy:

- Two Foxglove studio container with a default layout using a configmap (two different ones for the two robots)
- Two Foxglove bridge and Zenoh bridge containers colocated with Foxglove studio container for easier communication
- A Zenoh Router
- A Simple Flask webapp
- An nginx webserver to host the Thing description
- Two VOs corresponding to the robots with an a nginx webserver colocated for file access
- A cVO configured to connect to the two VOs deployed

After the deployment you can reach the Foxglove Studio (using Chrome webbrowser) at https://foxglove-demo.robopaas.dev/ and open a websocket connection to the bridge to reach the ROS world towards this ingress wss://bridges.robopaas.dev 
Any ROS topic published to the same Zenoh Router can be shown in the Foxglove dashboard.

The Zenoh broker is going to used also to expose data from ROS to the VO and send actions from the VO (starting from the webapp) to the tb2

The flask webapp GUI can be reached over a K8s ingress at flask-gui.robopaas.dev

N.B. You might need to change the NodePorts used!