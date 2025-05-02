# Simple example
1. To run the example first create a virtual environment in python (the location of the virtual environment doesn't matter):
```bash
$ python -m venv vo-wot-env
```

2. Activate the environment:
```bash
$ source vo-wot-env/bin/activate
```

3. Install the python package. To install the local development version instead along with the `tests` and `docs` dependencies, you need to run:
```bash
(vo-wot-env)$ pip install -U -e .[tests,docs]
```

4. Run the example. The python package provides a cli named `vo-wot`. To use it run:
```bash
vo-wot/examples/sqlite(vo-wot-env)$ vo-wot -t td.json -f config.yaml app.py
```

Alternatively the module can be directly called from the current directory using:
```bash
vo-wot/examples/sqlite(vo-wot-env)$ python ../../wotpy/cli/cli.py -t td.json -f config.yaml app.py
```
Afterwards the Virtual Object will be running and be ready to serve requests.

5. Run the client:
```bash
vo-wot/examples/sqlite(vo-wot-env)$ python client.py
```
