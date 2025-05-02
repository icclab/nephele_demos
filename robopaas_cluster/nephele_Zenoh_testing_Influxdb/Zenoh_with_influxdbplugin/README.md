# Zenoh Router with ROS2DDS and Influxdb plugin to Monitor ROS2 topics with Grafana

## About

In this branch we test the Influxdb storage backend support of the Zenoh router through its plugin. In particular we will focus on v2 of the Influxdb and the corresponding Zenoh plugin https://github.com/eclipse-zenoh/zenoh-backend-influxdb/tree/main/v2

We provide a basic setup of an Influxdb storage on our kubernetes cluster through a deployment and a Grafana deployment to monitor the data stored on the Influxdb.
The idea is that the Zenoh router automatically stores data on the Influxdb instance based on  the given configuration. The Grafana dashboards can then be used to display the stored data. 
Note that for ROS2 topics these will be stored mostly as strings in the db, so some prework is needed to publish the data to store and monitor. We will show an example of this using the code provided in this repository  https://github.com/iwatake2222/ros2_monitor_grafana In this case the data is pushed directly to the Influxdb, but the same applies for when going over the Zenoh router instead.


### Zenoh router with Influxdb library

Under the  Zenoh_with_influxdbplugin folder you find a Dockerfile for creating an container image of the Zenoh router with the Influxdb libraries. The image can then be pushed on Dockerhub. 

To make it work I had to use the 0.11 version of the router and the plugin, important is that the two versions match. The libraries can be downloaded here: https://download.eclipse.org/zenoh/zenoh-backend-influxdb/0.11.0/ and in particular the musl version is required for deployments on kubernetes. 

In a subfolder also the most recent version is available, but this wasn't working. 

### Influxdb deployment on Kubernetes

The Zenoh router will try to connect to an Influxdb database as storage backend. This storage needs to exist otherwise a failure occurs. So first deploy the Influxdb:

```sh
kubectl apply -f influxdb_v2.yaml -n <your-namespace>
```

The influxdb_v2.yaml file is under the k8s_deployment folder. This will create a deployment with initial access settings, a persistent volume claim and a service (loadbalancer). Note, that Influxdb is working on port 8086 which needs to be opened on the hosting VMs in Openstack.

Now you can access with the browser to the Influxdb with the initial username and passwor. Here you create an API token for read/write access to ALL your buckets in the db. Note this down as this will be needed for the Zenoh router plugin configuration file.

### Zenoh router deployment on Kubernetes with Influxdb plugin

The Docker image created in the first step will now be used to deploy the Zenoh router with the Influxdb plugin. The zenoh-router.yaml file will start the router with a configuration file zenoh_config.json5  defining the connection to the Influxdb, which topics to store and how, which database to use (see under k8s_deployment folder).

In our setting we have general deployment through kustomization for all the components we need as defined in the kustomization.yaml file.

```sh
kubectl apply -k k8s_deployment/ -n <your-namespace>
```

You can now run some ros project on any environment connected to the Zenoh router and for those topics matching the configuration you will see them stored in the Influxdb.

### Grafana deployment on Kubernetes 

To monitor data stored in the Influxdb we deploy a Grafana container. This is accessible on port 3000 which needs to be opened on the VM hosting the Kubernetes cluster (initial user and passord are admin:admin).

```sh
kubectl apply -f grafana.yaml -n <your-namespace>
```
Once on Grafana you can follow the instructions reported here https://docs.influxdata.com/influxdb/v2/tools/grafana/?t=InfluxQL to connect to your Influxdb. If everything works you should see your data and you can define your queries/dashboards accordingly. In short:


- Login to Grafana at: http://your-ip-address:3000  (admin, admin)
- Configure datasource
  - `Configuration` -> `Data sources` -> `Add data source`
  - Select `InfluxDB`, and process the following settings, then click `Save & Test`
    - Default: checked
    - Query Language: Flux
    - URL: http://your-ip-address:8086
    - (User: your-user)
    - (Password: your-password)
    - Organization: your-org
    - Token: your-super-secret-auth-token
    - Default Bucket: your-bucket



### ROS topics adaptation

The ROS topics are mostly stored as strings and need some reorganization. For instance ad-hoc desinged publishers can be defined and the Zenoh Influxdb plugin configured to store only these topics. As an example we can use the code under ros2_monitor_grafana/src (from https://github.com/iwatake2222/ros2_monitor_grafana) which however connects directly to the Influxdb without going over the Zenoh router and using the Zenoh-influxdb plugin. To test this code do the following:

```sh
pip3 install influxdb_client
```


```sh
token=my-super-secret-auth-token
org=my-org
url=http://ipaddress:8086
bucket_name=my-bucket

# Start monitoring ROS 2 topics and uploading data
python3 src/main.py --token=$token --org=$org --url=$url --bucket_name=$bucket_name
```

To configure the dashboard access your grafana instance


- Login to Grafana
- Configure dashboards
  - `Dashboards` -> `Browse` -> `New` -> `Import`
  - `Upload JSON file`
    - Select  `k8s_deployment/grafana_dashboard_topicmonitor.json`
    - Click `Import`


