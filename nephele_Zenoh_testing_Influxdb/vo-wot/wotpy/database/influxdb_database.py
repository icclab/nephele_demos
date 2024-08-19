#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Class that handles InfluxDB database operations.
"""

import json
import time

import influxdb_client

from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS


class InfluxDB:
    """Class that handles InfluxDB database operations."""

    def __init__(self, url, org, token):
        self.client = InfluxDBClient(url=url, org=org, token=token)

    def is_reachable(self):
        """Tries a certain number of times to connect to the InfluxDB database
        and raises a ConnectionError if the connection cannot be established."""

        max_retries = 15
        retry_interval = 10
        for retry in range(max_retries):
            if not self.client.ping():
                if retry == max_retries - 1:
                    return False
                time.sleep(retry_interval)
        return True

    def init_apis(self):
        """Initializes the InfluxDB APIs."""

        self.buckets_api = self.client.buckets_api()
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
        self.query_api = self.client.query_api()

    def close_apis(self):
        """Closes the APIs when done."""

        self.write_api.close()

    def write_point(self, key, value):
        """Writes the value in the specified bucket creating the bucket if it
        doesn't exist."""

        if isinstance(value, list):
            value = json.dumps(value)

        point = influxdb_client.Point(key).field("value", value)

        if not self.buckets_api.find_bucket_by_name(key):
            self.buckets_api.create_bucket(bucket_name=key)
        self.write_api.write(bucket=key, record=point)

    def execute_query(self, query):
        """Executes the input query and returns its output."""\

        return self.query_api.query(org="wot", query=query)
