#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
CLI for the WoT runtime that handles servient configuration and execution of WoT runtime scripts.
"""

import argparse
import asyncio
import json
import logging
import time
import yaml
import importlib.util

from tornado.httpclient import HTTPClientError
from tornado.ioloop import PeriodicCallback

from wotpy.functions import functions
from wotpy.cli.default_servient import DefaultServient
from wotpy.protocols.netconf.utils import inject_netconf_properties
from wotpy.utils.proxy import build_prop_read_proxy, build_prop_write_proxy, \
    build_action_invoke_proxy, subscribe_event


def create_proxy_functions(consumed_vos, proxy_dict, exposed_thing):
    """
    Creates proxy functions that propagate interactions with properties, actions
    and events to another thing. The operations that are proxied are:

    - Properties:
        - property read
        - property write
    - Actions:
        - action invocation
    - Events:
        - on_next function
        - on_completed function
        - on_error function
    """

    if "propertiesMap" in proxy_dict and proxy_dict["propertiesMap"] is not None:
        for proprty, target_vo in proxy_dict["propertiesMap"].items():
            exposed_thing.set_property_read_handler(
                proprty,
                build_prop_read_proxy(consumed_vos[target_vo], proprty)
            )
            exposed_thing.set_property_write_handler(
                proprty,
                build_prop_write_proxy(consumed_vos[target_vo], proprty)
            )

    if "actionsMap" in proxy_dict and proxy_dict["actionsMap"] is not None:
        for action, target_vo in proxy_dict["actionsMap"].items():
            exposed_thing.set_action_handler(
                action,
                build_action_invoke_proxy(consumed_vos[target_vo], action)
            )

    if "eventsMap" in proxy_dict and proxy_dict["eventsMap"] is not None:
        for event, target_vo in proxy_dict["eventsMap"].items():
            subscribe_event(consumed_vos[target_vo], exposed_thing, event)


async def consume_vos(consumed_vos_dict, wot, credentials_dict):
    """Consume VOs from the given URLs."""

    consumed_vos = {}

    for vo_name, vo_dict in consumed_vos_dict.items():
        url = vo_dict["url"]
        max_retries = 30
        retry_interval = 10
        for retry in range(max_retries):
            try:
                consumed_vos[vo_name] = await wot.consume_from_url(url, credentials_dict)
            except (ConnectionRefusedError, HTTPClientError) as exception:
                if retry == max_retries - 1:
                    raise ConnectionError(
                        f"Connection failed after {max_retries} retries"
                    ) from exception
                time.sleep(retry_interval)

    return consumed_vos


async def subscribe_remote_events(consumed_vos, consumed_vos_dict, module):
    """Maps remote events/property changes to the corresponding user-defined functions."""

    for vo_name, vo_dict in consumed_vos_dict.items():
        events = vo_dict.get("events", [])
        for event in events:
            on_next_handler_name = event + "_" + vo_name + "_on_next"
            on_completed_handler_name = event + "_" + vo_name + "_on_completed"
            on_error_handler_name = event + "_" + vo_name + "_on_error"

            on_next_handler = getattr(module, on_next_handler_name, None)
            on_completed_handler = getattr(module, on_completed_handler_name, None)
            on_error_handler = getattr(module, on_error_handler_name, None)

            consumed_vos[vo_name].events[event].subscribe(
                on_next=on_next_handler,
                on_completed=on_completed_handler,
                on_error=on_error_handler
            )
        property_changes = vo_dict.get("propertyChanges", [])
        for proprty in property_changes:
            on_next_handler_name = proprty + "_" + vo_name + "_on_next"
            on_completed_handler_name = proprty + "_" + vo_name + "_on_completed"
            on_error_handler_name = proprty + "_" + vo_name + "_on_error"

            on_next_handler = getattr(module, on_next_handler_name, None)
            on_completed_handler = getattr(module, on_completed_handler_name, None)
            on_error_handler = getattr(module, on_error_handler_name, None)

            consumed_vos[vo_name].properties[proprty].subscribe(
                on_next=on_next_handler,
                on_completed=on_completed_handler,
                on_error=on_error_handler
            )


async def map_user_defined_code(TD, exposed_thing, module):
    """
    Maps the following user-defined code to the corresponding WoT constructs:

    - Properties:
        - initial values
        - property read handler
        - property write handler
        - property changes:
            - on_next function
            - on_completed function
            - on_error function
    - Actions:
        - action invocation handler
    - Events:
        - on_next function
        - on_completed function
        - on_error function
    """

    for proprty in TD.get("properties", {}):
        property_init_name = proprty + "_init"
        property_init_value = getattr(module, property_init_name, None)
        if property_init_value is not None:
            await exposed_thing.properties[proprty].write(property_init_value)

        property_read_handler_name = proprty + "_read_handler"
        property_write_handler_name = proprty + "_write_handler"
        target_read_function = getattr(module, property_read_handler_name, None)
        target_write_function = getattr(module, property_write_handler_name, None)

        if target_read_function is not None:
            exposed_thing.set_property_read_handler(proprty, target_read_function)
        if target_write_function is not None:
            exposed_thing.set_property_write_handler(proprty, target_write_function)

        on_next_handler_name = proprty + "_on_next"
        on_completed_handler_name = proprty + "_on_completed"
        on_error_handler_name = proprty + "_on_error"

        on_next_handler = getattr(module, on_next_handler_name, None)
        on_completed_handler = getattr(module, on_completed_handler_name, None)
        on_error_handler = getattr(module, on_error_handler_name, None)

        if on_next_handler is not None:
            exposed_thing.properties[proprty].subscribe(
                on_next=on_next_handler,
                on_completed=on_completed_handler,
                on_error=on_error_handler
            )

    for action in TD.get("actions", {}):
        action_handler_name = action + "_handler"
        target_function = getattr(module, action_handler_name, None)

        if target_function is not None:
            exposed_thing.set_action_handler(action, target_function)

    for event in TD.get("events", {}):
        on_next_handler_name = event + "_on_next"
        on_completed_handler_name = event + "_on_completed"
        on_error_handler_name = event + "_on_error"

        on_next_handler = getattr(module, on_next_handler_name, None)
        on_completed_handler = getattr(module, on_completed_handler_name, None)
        on_error_handler = getattr(module, on_error_handler_name, None)

        if on_next_handler is not None:
            exposed_thing.events[event].subscribe(
                on_next=on_next_handler,
                on_completed=on_completed_handler,
                on_error=on_error_handler
            )


def schedule_periodic_functions(periodic_function_data, module):
    """Schedules functions to run periodically."""

    for periodic_function, periodicity in periodic_function_data.items():
        function = getattr(module, periodic_function, None)
        if function is not None:
            periodic_callback = PeriodicCallback(function, periodicity)
            periodic_callback.start()
        else:
            raise TypeError(f"Function {function} definition needs to be declared")


def inject_generic_function(generic_function_data, module):
    """Inject generic function in the user-defined module's functions"""

    for generic_function in generic_function_data:
        function = getattr(functions, generic_function)
        setattr(module, generic_function, function)


async def run_script(thing_description_path, script_path, config_path):
    """Creates a Servient based on the config file and initializes the WoT runtime"""

    config = {}
    if config_path is not None:
        with open(config_path, "r") as config_file:
            config = yaml.safe_load(config_file)

    default_servient = DefaultServient(config)
    wot = await default_servient.start()

    # Dynamically loads the python script with the user-defined code
    spec = importlib.util.spec_from_file_location("wot_script", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    with open(thing_description_path, "r") as thing_description:
        TD = json.load(thing_description)

    exposed_thing = wot.produce(json.dumps(TD))

    http_sb_credentials = default_servient.config["bindingSB"]["securitySB"]["securitySBHTTP"]
    try:
        holder_url = http_sb_credentials["holderUrl"]
        requester = http_sb_credentials["requester"]
        if holder_url is not None and requester is not None:
            credentials_dict = {
                "holder_url": holder_url,
                "requester": requester
            }
        else:
            credentials_dict = None
    except KeyError:
        credentials_dict = None
    consumed_vos_dict = default_servient.config.get("consumedVOs", {})
    consumed_vos = await consume_vos(consumed_vos_dict, wot, credentials_dict)
    await subscribe_remote_events(consumed_vos, consumed_vos_dict, module)

    proxy_data = default_servient.config.get("proxy", {})
    create_proxy_functions(consumed_vos, proxy_data, exposed_thing)

    generic_function_data = default_servient.config.get("genericFunction", [])
    inject_generic_function(generic_function_data, module)

    if default_servient.config["bindingNB"]["netconf"]["enabled"]:
        inject_netconf_properties(
            exposed_thing,
            default_servient.config["bindingNB"]["netconf"]["schedulerURL"],
            default_servient.config["bindingNB"]["netconf"]["netconfServerURL"]
        )

    # Make instances available to user-defined code
    module.exposed_thing = exposed_thing
    module.consumed_vos = consumed_vos

    await map_user_defined_code(TD, exposed_thing, module)

    exposed_thing.expose()

    periodic_function_data = default_servient.config.get("periodicFunction", {})
    schedule_periodic_functions(periodic_function_data, module)


def setup_logging():
    logging.basicConfig()
    LOGGER = logging.getLogger()
    LOGGER.setLevel(logging.INFO)


def main():
    setup_logging()
    parser = argparse.ArgumentParser(description="Run a WoT script optionally preconfigured by a config file.")
    parser.add_argument("script", help="user python script file")
    parser.add_argument("-f", "--config-file", help="path to the configuration file")
    parser.add_argument("-t", "--thing-description", help="path to the thing description")
    args = parser.parse_args()

    loop = asyncio.get_event_loop()
    loop.create_task(run_script(args.thing_description, args.script, args.config_file))
    loop.run_forever()


if __name__ == "__main__":
    main()
