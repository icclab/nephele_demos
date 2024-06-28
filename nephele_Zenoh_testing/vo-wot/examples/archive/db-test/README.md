# How to test saving properties to an Influxdb database
## Run with Kubernetes
1. Build the docker image located at the top level directory of this repo:
```bash
docker build -t wotpy .
```
If using minikube to load the image to minikube's docker also run:
```bash
minikube image load wotpy
```
To mount the script inside minikube leave this command running inside this directory:
```bash
minikube mount $PWD:/mnt
```
2. Execute the deployment and create the Kubernetes service by running:
```bash
kubectl apply -f deployment.yaml
```
3. Port forward the service along with the necessary ports (it doesn't work for the coap udp port):
```bash
kubectl port-forward svc/wotpy 8080 8081 9090
```
4. Execute the `test-client.py`
## Run with docker
Run the influxdb image and initialize it with the specified credentials:
```bash
docker run --rm --name influxdb -p 8086:8086 \
      -e DOCKER_INFLUXDB_INIT_MODE=setup \
      -e DOCKER_INFLUXDB_INIT_USERNAME=my-username \
      -e DOCKER_INFLUXDB_INIT_PASSWORD=my-password \
      -e DOCKER_INFLUXDB_INIT_ORG=wot \
      -e DOCKER_INFLUXDB_INIT_BUCKET=my-bucket \
      -e DOCKER_INFLUXDB_INIT_RETENTION=1w \
      -e DOCKER_INFLUXDB_INIT_ADMIN_TOKEN=my-token \
      influxdb:2.6.0
```
## Install influxdb and enable it as a service
1. Download Influxdb (Arch Linux):
    ```bash
    pacman -Syu influxdb
    ```
2. Start the database service:
    ```bash
    sudo systemctl start influxdb
    ```
3. Create a user with the following credentials and the given token or do it from the UI by accessing the URL in `localhost:8086`:
    ```bash
    curl -i -X POST http://localhost:8086/api/v2/setup -H 'accept: application/json' \
        -d '{
                "username": "my-username",
                "password": "my-password",
                "org": "wot",
                "bucket": "my-bucket",
                "token": "my-token"
            }'
    ```

