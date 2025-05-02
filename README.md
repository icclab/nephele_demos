# nephele working repo

You will find under nephele_cluster and robopaas_cluster the UC1 Nephle demo files, respectively for deployment on the nephele-project Kubernetes cluster and on the robopaas cluster. This includes the right IP addresses configuration and ingresses in all the configuration files (e.g., Zenoh bridge in all containers). Each container is built with a specific tag which is then used in the deployment.


To deploy all the components you can run:

kubectl apply -k k8s_deployment/ -n <namespace>




Under "Helm_charts" you can find the Helm charts for the application components (configured for the nephele cluster). Specific robot configuration and starting scripts are under "config_and_launchfiles".


