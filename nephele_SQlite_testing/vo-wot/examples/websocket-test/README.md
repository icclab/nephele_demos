# WebSocket Protocol Test
To run the VO first install the python package. In the root directory of the repository do:
```bash
$ python -m venv .venv
$ source .venv/bin/activate
(venv)$ pip install -U -e .[tests,docs]
```
Afterwards in this directory do:
```bash
examples/websocket-test(venv)$ vo-wot -f config.yaml -t td.json app.py
```
The VO should be running and waiting for requests. The WebSocket server port is 9393 as defined in the `config.yaml` file.

To test the client for this VO simply run the `client.py` file in a different terminal:
```bash
examples/websocket-test(venv)$ python client.py
```
The client simply writes the value of a dummy property and then reads it again.
In this file a Servient class is created and creates the necessary clients to interact with the Virtual Object.

In case the user needs to communicate with the VO directly without the use of the VO Stack, the API is shown [here](https://netmode.gitlab.io/vo-wot/websockets.html).