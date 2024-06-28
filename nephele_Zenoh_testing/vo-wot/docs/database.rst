VO data persistency
===================

This page includes what kind of data VO may store internally:

* **SQLite** database schema
* **InfluxDB** database schema

SQLite database schema
----------------------
This section explains the relational schema of the VO in SQLite.

.. list-table:: SQLite database tables

	* - VO status (self-check)
	* - Status of IoT device (alive, loss of connectivity)
	* - Observer list
	* - Redirect IPs
	* - Device IP

The SQLite database schema is as follows (subject to change):

.. code-block:: sql

	CREATE TABLE IF NOT EXISTS vo_status (
		id INTEGER PRIMARY KEY,
		timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
		status INTEGER
	);
	CREATE TABLE IF NOT EXISTS device_status (
		id INTEGER PRIMARY KEY,
		timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
		status INTEGER
	);
	CREATE TABLE IF NOT EXISTS observer (
		id INTEGER PRIMARY KEY,
		ip TEXT
	);
	CREATE TABLE IF NOT EXISTS redirect_ip (
		id INTEGER PRIMARY KEY,
		ip TEXT
	);
	CREATE TABLE IF NOT EXISTS info (
		id INTEGER PRIMARY KEY,
		device_ip TEXT
	);

InfluxDB database schema
------------------------
This section explains how time series data of the VO are stored in InfluxDB.

.. list-table:: InfluxDB database buckets

	* - **Properties**
	  - Saved as key-value pairs. The key is the name

	    of the property and the value is the value of the property.
	* - **Actions**
	  - Saved as key-value pairs. The key is "action" and

	    the value is the function invocation data.
	* - **Events**
	  - Saved as key-value pairs. The key is "event" and

	    the value is the event data.

Actions and events are stored in the database in order to log function invocations
and all emitted events.

.. note::
	To start up an InfluxDB instance locally, docker can be used:

	.. code-block:: bash

		docker run --rm --name influxdb -p 8086:8086 \
			-e DOCKER_INFLUXDB_INIT_MODE=setup \
			-e DOCKER_INFLUXDB_INIT_USERNAME=my-username \
			-e DOCKER_INFLUXDB_INIT_PASSWORD=my-password \
			-e DOCKER_INFLUXDB_INIT_ORG=wot \
			-e DOCKER_INFLUXDB_INIT_BUCKET=my-bucket \
			-e DOCKER_INFLUXDB_INIT_RETENTION=1w \
			-e DOCKER_INFLUXDB_INIT_ADMIN_TOKEN=my-token \
			influxdb:2.6.0

Interaction with the databases
------------------------------

Using the ``exposed_thing`` object the developer has access to the InfluxDB query API as well. For example:

.. code-block:: py

    tables = exposed_thing.servient.influxdb.execute_query(query)

will return the result of the developer's query where ``query`` is a valid InfluxDB query.

Similarly, by using the convenience functions for the SQLite database,
the user can directly insert or query data from the database:

.. code-block:: py

    exposed_thing.servient.sqlite_db.insert_data(table_name, data)
    exposed_thing.servient.sqlite_db.execute_query(query)