#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Generic function definitions.
"""

import time
import datetime

import numpy as np
import pmdarima as pm
import tornado.httpclient


async def forecasting(exposed_thing, property_name):
    servient = exposed_thing.servient

    query = 'from(bucket:"{}")\
    |> range(start: -10m)'.format((property_name)) #TODO change limit of query
    tables = servient.influxdb.execute_query(query)

    # Serialize to values
    output = tables.to_values(columns=["_value"])
    flat_list = [item for sublist in output for item in sublist]

    arr = np.array(flat_list)
    model = pm.auto_arima(arr, start_p=1, start_q=1, test="adf",       # use adftest to find optimal "d"
                          max_p=3, max_q=3, # maximum p and q
                          m=1,              # frequency of series
                          d=None,           # let model determine "d"
                          seasonal=False,   # No Seasonality
                          start_P=0,
                          D=0,
                          trace=False,
                          error_action="ignore",
                          suppress_warnings=True,
                          stepwise=True)
    predicted_value = model.predict(n_periods=1)

    return float(predicted_value[0])

async def mean_value(exposed_thing, property_name, horizon):
    """Queries the influxdb database and averages the data
    for the given property."""

    servient = exposed_thing.servient

    query = 'from(bucket:"{}")\
        |> range(start: -10m)\
        |> tail(n:{})\
        |> mean()'.format(property_name, horizon) #TODO change limit of query

    tables = servient.influxdb.execute_query(query)
    # Serialize to values
    output = tables.to_values(columns=["_value"])
    flat_list = [item for sublist in output for item in sublist]
    return flat_list[0]

async def vo_status(exposed_thing, id):
    """Attempts to access the catalogue port of the VO and if successful
    inserts a row containing an integer (0 for failure or 1 for success)
    in the corresponding table. Returns all rows from the `vo_status` table."""

    servient = exposed_thing.servient

    timestamp = time.time()
    datetime_format = datetime.datetime.fromtimestamp(timestamp)

    try:
        http_client = tornado.httpclient.AsyncHTTPClient()
        url = f"http://localhost:{servient.catalogue_port}"
        await http_client.fetch(url)
    except Exception as exception:
        print(f"Connection to VO Error: {exception}")
        exposed_thing.emit_event("VO_Connection_Error")
        servient.sqlite_db.insert_data("vo_status", (id, datetime_format, 0))
    else:
        servient.sqlite_db.insert_data("vo_status", (id, datetime_format, 1))

    return servient.sqlite_db.execute_query("SELECT * FROM vo_status")
    # TODO initialize event on cli if status_vo is enabled
    # TODO change localhost


# Deployment type A
async def device_status(exposed_thing, device_catalogue_url, id):
    """Attempts to access the catalogue port of the device and if successful
    inserts a row containing an integer (0 for failure or 1 for success)
    in the corresponding table. Returns all rows from the `device_status` table."""

    servient = exposed_thing.servient

    timestamp = time.time()
    datetime_format = datetime.datetime.fromtimestamp(timestamp)
    try:
        http_client = tornado.httpclient.AsyncHTTPClient()
        await http_client.fetch(device_catalogue_url)
    except Exception as exception:
        print(f"Connection to Device Error: {exception}")
        exposed_thing.emit_event("Device_Connection_Error", f"Device_Connection_Error: {False}%")
        servient.sqlite_db.insert_data("device_status", (id, datetime_format, 0))
    else:
        servient.sqlite_db.insert_data("device_status", (id, datetime_format, 1))
    return servient.sqlite_db.execute_query("SELECT * FROM device_status")
    # TODO initialize event on cli if status_device is enabled
    # TODO device must have an id or name
    # TODO think of a way to implement this in deployment type B