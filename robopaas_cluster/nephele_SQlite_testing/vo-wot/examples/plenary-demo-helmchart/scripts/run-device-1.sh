SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

python ${SCRIPT_DIR}/../../../wotpy/cli/cli.py -f ${SCRIPT_DIR}/../device_1.yaml -t ${SCRIPT_DIR}/../device_1_td.json ${SCRIPT_DIR}/../device_1.py
