This repository contains the deployment files and container image creation files for Use Case 1 demonstration.


Use the following command to deploy on the kubernetes cluster:

``
kubectl apply -k k8s_deployment -n <namespace>
``

You will deploy:

- A  Flask GUI
- One Foxglove studio container with a default layout using a configmap
- One Foxglove bridge and Zenoh bridge containers colocated with Foxglove studio container for easier communication
- A Zenoh Router
- An nginx webserver to host the Thing description
- Two VOs corresponding to the robots with an a nginx webserver colocated for file access
- A cVO configured to connect to the two VOs deployed
- Two rosbag saver containers for the two robots
- A roboflow container for person and liquid detection
- A frontend API for sensor network management
- Backend API for sensor network management

After the deployment you can reach the Foxglove Studio (using Chrome webbrowser) and open a websocket connection to the bridge to reach the ROS world.
Any ROS topic published to the same Zenoh Router can be shown in the Foxglove dashboard.

The Zenoh broker is going to used also to expose data from ROS to the VO and send actions from the VO (starting from the webapp) to the robots.


