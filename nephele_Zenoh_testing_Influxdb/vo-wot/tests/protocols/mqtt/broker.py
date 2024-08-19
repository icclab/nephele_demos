#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import logging
import os

from amqtt.client import MQTTClient, ConnectException

from wotpy.protocols.mqtt.enums import MQTTCodesACK

ENV_BROKER_URL = "WOTPY_TESTS_MQTT_BROKER_URL"
BROKER_SKIP_REASON = "The test MQTT broker is offline"


def get_test_broker_url():
    """Returns the MQTT broker URL defined in the environment."""

    return os.environ.get(ENV_BROKER_URL, None)


def is_test_broker_online():
    """Returns True if the MQTT broker defined in the environment is online."""

    async def check_conn():
        broker_url = get_test_broker_url()

        if not broker_url:
            logging.warning("Undefined MQTT broker URL")
            return False

        try:
            amqtt_client = MQTTClient()
            ack_con = await amqtt_client.connect(broker_url)
            if ack_con != MQTTCodesACK.CON_OK:
                logging.warning("Error ACK on MQTT broker connection: {}".format(ack_con))
                return False
        except ConnectException as ex:
            logging.warning("MQTT broker connection error: {}".format(ex))
            return False

        return True

    loop = asyncio.get_event_loop_policy().get_event_loop()
    conn_ok = loop.run_until_complete(check_conn())

    if conn_ok is False:
        logging.warning(
            "Couldn't connect to the test MQTT broker. "
            "Please check the {} variable".format(ENV_BROKER_URL))

    return conn_ok
