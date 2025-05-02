#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Utility functions for NETCONF-related functionalities"""

import json

import tornado.httpclient

JSON_HEADERS = {"Content-Type": "application/json"}
XML_HEADERS = {"Content-Type": "application/xml"}


async def call_scheduler(scheduler_url, flow_data, network_data):
    flow_data = json.loads(flow_data)
    network_data = json.loads(network_data)
    if "network" not in network_data or "flows" not in flow_data:
        return None

    # Extract nodes and edges from network_data
    topology = network_data.get("network", {}).get("topology", {})
    nodes = topology.get("nodes", [])
    edges = topology.get("edges", [])

    # Combine network and flow data into flow_request_data
    flow_request_data = {
        "network": {
            "topology": {
                "nodes": nodes,
                "edges": edges
            }
        },
        "flows": flow_data["flows"]
    }

    api_url = f"{scheduler_url}/schedule/taprio"
    http_client = tornado.httpclient.AsyncHTTPClient()
    http_request = tornado.httpclient.HTTPRequest(
        api_url,
        method="GET",
        headers=JSON_HEADERS,
        body=json.dumps(flow_request_data),
        allow_nonstandard_methods=True
    )

    response = await http_client.fetch(http_request)
    return json.loads(response.body)


async def configure_taprio_schedules(netconf_server_url, dev_name, schedule_list, cmd="create"):
    """
    Configures TAPRIO schedules on a network device with
    a command `cmd` using HTTPS POST.
    """

    config = f"""
    <rpc message-id="101"
    xmlns="urn:ietf:params:xml:ns:netconf:base:1.0">
    <edit-config>
        <target>
            <running/>
        </target>
        <default-operation>none</default-operation>
        <test-option>test-then-set</test-option>
        <error-option>rollback-on-error</error-option>
        <config xmlns="urn:ietf:params:xml:ns:netconf:base:1.0">
            <taprio-schedules operation="{cmd}" xmlns="urn:ietf:params:xml:ns:yang:taprio-schedules">
                <schedules>
                    <device>{dev_name}</device>
                    <sched-entries>"""

    for schedule in schedule_list:
        config += f"""
                        <sched-entry>
                            <command>S</command>
                            <gatemask>{schedule['gatemask']}</gatemask>
                            <interval>{schedule['interval']}</interval>
                        </sched-entry>"""

    config += """
                    </sched-entries>
                </schedules>
            </taprio-schedules>
        </config>
    </edit-config>
    </rpc>"""  # Close the rpc tag here

    api_url = f"{netconf_server_url}/edit-config"
    http_client = tornado.httpclient.AsyncHTTPClient()
    http_request = tornado.httpclient.HTTPRequest(
        api_url,
        method="POST",
        headers=XML_HEADERS,
        body=config
    )
    await http_client.fetch(http_request)


async def call_netconf_server(netconf_server_url, schedule):
    """Configures the specified switches of the schedule"""

    for switch in schedule["switches"]:
        switch_id = switch["switch"]
        schedule_list = switch["interfaces"][0]["schedEntries"]
        await configure_taprio_schedules(netconf_server_url, switch_id, schedule_list)


def inject_netconf_properties(exposed_thing, scheduler_url, netconf_server_url):
    """Inject NETCONF related properties to an exposed thing"""

    async def flow_write_handler(value):
        flow = json.dumps(value)
        await exposed_thing._default_update_property_handler("netconf_flow", flow)
        network = await exposed_thing.read_property("netconf_network")
        if network is not None:
            schedule = await call_scheduler(scheduler_url, flow, network)
            if schedule is not None:
                await exposed_thing.write_property("netconf_schedule", schedule)

    async def network_write_handler(value):
        network = json.dumps(value)
        await exposed_thing._default_update_property_handler("netconf_network", network)
        flow = await exposed_thing.read_property("netconf_flow")
        if flow is not None:
            schedule = await call_scheduler(scheduler_url, flow, network)
            if schedule is not None:
                await exposed_thing.write_property("netconf_schedule", schedule)

    async def schedule_write_handler(value):
        await exposed_thing._default_update_property_handler("netconf_schedule", value)
        await call_netconf_server(netconf_server_url, value)

    prop_dict = {
        "type": "string",
        "observable": True
    }
    netconf_property_handlers = {
        "netconf_flow": flow_write_handler,
        "netconf_network": network_write_handler,
        "netconf_schedule": schedule_write_handler
    }
    for proprty, handler in netconf_property_handlers.items():
        exposed_thing.add_property(proprty, prop_dict)
        exposed_thing.set_property_write_handler(proprty, handler)
