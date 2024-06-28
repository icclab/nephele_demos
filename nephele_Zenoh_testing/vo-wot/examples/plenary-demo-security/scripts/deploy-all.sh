SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

kubectl apply -f ${SCRIPT_DIR}/../vo-1/deployment.yaml
kubectl apply -f ${SCRIPT_DIR}/../app-1/deployment.yaml
