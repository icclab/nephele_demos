Distributed Application Graph with VO full example
==================================================

This repository includes a complete example of a distributed application that contains a VO/cVO microservice(s).

This example depicts an application graph using two Virtual Objects and their two corresponding devices,
one Composite Virtual Object and one application component that consumes the data from the Virtual Objects.

.. image:: images/graph.png
      :width: 400

The specific Web of Things python scripts, the VO descriptors and the Kubernetes Deployments and Services are available in
the repo's `plenary-demo <https://gitlab.eclipse.org/eclipse-research-labs/nephele-project/vo-wot/-/tree/main/examples/plenary-demo>`__
directory. The README file of that directory contains deployment instructions.

A helm chart with templated Kubernetes objects is also available inside the
`plenary-demo-helmchart <https://gitlab.eclipse.org/eclipse-research-labs/nephele-project/vo-wot/-/tree/main/examples/plenary-demo-helmchart>`__
directory.

Graph explanation
-----------------
In the device layer there are two devices. One that reports Temperature and Humidity data (Device 1) and one that reports the
presence of smoke (Device 2). Each of these two devices have a virtual counterpart namely the Virtual Object 1 (VO1) and
the Virtual Object 2 (VO2). The composite Virtual Object (cVO1) aggregates the information of the two Virtual Objects.
In particular it subscribes to an event in VO2 about the presence of smoke and once this event is emitted, it makes
a request to the VO1 to ask about the current reading of the temperature. If the temperature reported is above
a certain threshold, then the "fire" property is set. Lastly, the component 1 in the image represents an application
that consumes the information of the cVO1 exposing an endpoint ``/fire`` that reports the presence of fire or not
along with some additional endpoints that do averaging and forecasting on the VO1's values directly.

Descriptors
-----------
The files used to deploy the graph are as follows:

* Device 1:

    * Thing Description:

      .. code:: json

        {
            "title": "device1",
            "id": "urn:dev:wot:plenary:device1",
            "description": "'Lille Plenary Meeting Descriptor for Device 1 (with computing capabilities).'",
            "securityDefinitions": {
                "basic_sec": {
                    "scheme": "basic"
                }
            },
            "security": "basic_sec",
            "@context": [
                "https://www.w3.org/2022/wot/td/v1.1"
            ],
            "properties": {
                "temperature": {
                    "type": "integer"
                },
                "humidity": {
                    "type": "integer"
                }
            },
            "actions": {
                "currentValues": {
                    "description": "Returns the current values"
                }
            }
        }

    * VO Descriptor:

      .. code:: yaml

        # ========================================
        # Lille Plenary Meeting Device1 Descriptor
        # ========================================
        name: device1
        type: VO
        resourceType:
        specification: WoT
        version: 1.1
        deploymentType: B
        catalogue: 9090
        bindingNB:
        bindingModeNB: [M, H]
        hostname: 192.168.49.1
        ports:
            httpPort: 8080
        brokerIP: "mqtt://192.168.49.1:1883"
        securityNB:
            securityScheme: basic
            username: user
            password: pass

    * Python script:

      .. code:: python

        import random

        LOWER_VALUE = 30
        UPPER_VALUE = 50

        async def temperature_read_handler():
            return random.randint(LOWER_VALUE, UPPER_VALUE)

        humidity_read_handler = temperature_read_handler

        async def currentValues_handler(params):
            return {
                'result': True,
                'message': {
                    "temperature": random.randint(LOWER_VALUE,UPPER_VALUE),
                    "humidity": random.randint(LOWER_VALUE,UPPER_VALUE)
                }
            }

        temperature_init = LOWER_VALUE

* Device 2:

    * Thing Description: N/A


    * VO Descriptor: N/A

    * Python script (only used to send data to VO2):

      .. code:: python

        #!/usr/bin/env python
        # -*- coding: utf-8 -*-

        import asyncio
        import logging
        import json
        import aiocoap
        from aiocoap import *

        logging.basicConfig()
        LOGGER = logging.getLogger()
        LOGGER.setLevel(logging.INFO)

        async def main():
            protocol = await Context.create_client_context()
            VO_2_url = "192.168.49.2:30012"
            payload = json.dumps({"value" : "1"}).encode("utf-8")
            request = Message(code=aiocoap.Code.PUT, payload=payload, uri=f'coap://{VO_2_url}/property?thing=vo2&name=smoke', )
            try:
                response = await protocol.request(request).response
            except Exception as e:
                print('Failed to fetch resource:')
                print(e)
            else:
                print('Result: %s\n%r'%(response.code, response.payload))

        if __name__ == '__main__':
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(main())


* VO1:

    * Thing Description:

      .. code:: json

        {
            "title": "vo1",
            "id": "urn:dev:wot:plenary:VO1",
            "description": "Lille Plenary Meeting Descriptor for VO1 for Device 1.",
            "securityDefinitions": {
                "bearer_sc": {
                    "scheme": "bearer"
                }
            },
            "security": "bearer_sc",
            "@context": [
                "https://www.w3.org/2022/wot/td/v1.1"
            ],
            "properties": {
                "temperature": {
                    "type": "integer"
                },
                "humidity": {
                    "type": "integer"
                },
                "average_values":{
                    "type": "object",
                        "average_temperature": {
                            "type": "integer"
                        },
                        "average_humidity": {
                            "type": "integer"
                        }
                }
            },
            "actions": {
                "currentValues": {
                    "description": "Returns current Values for temperature and humidity from Device 1 through the Consumed Thing"
                },
                "averageData": {
                    "description": "Calculate the average_values property from the latest 10 values from InfluxDB"
                }
            }
        }

    * VO Descriptor:

      .. code:: yaml

        # ====================================
        # Lille Plenary Meeting VO1 Descriptor
        # ====================================
        name: vo1
        type: VO
        resourceType:
        specification: WoT
        version: 1.1
        deploymentType: A
        catalogue: 9090
        bindingNB:
        bindingModeNB: [H]
        hostname: vo1
        ports:
            httpPort: 8080
        securityNB:
            securityScheme: bearer
            token: token
        bindingSB:
        bindingModeSB: [H, M]
        brokerIP: mqtt://192.168.49.1:1883
        securitySB:
            securitySBMQTT:
                securityScheme: basic
                username: user
                password: pass
            securitySBHTTP:
                securityScheme: basic
                username: user
                password: pass
        databaseConfig:
        timeseriesDB:
            influxDB: enabled
            address: "http://wotpy-influxdb-vo1:8086"
            dbUser: my-username
            dbPass: my-password
            dbToken: my-token
        persistentDB:
            SQLite: enabled
        genericFunction: [mean_value, vo_status, device_status]
        periodicFunction:
        check: 1000
        read_property_from_device_1: 5000
        consumedVOs:
        device1:
            url: "http://192.168.49.1:9090/device1"
        proxy:
        propertiesMap:
            temperature: device1
            humidity: device1
        actionsMap:
            currentValues: device1


    * Python script:

      .. code:: python

        #!/usr/bin/env python
        # -*- coding: utf-8 -*-

        from wotpy.functions.functions import mean_value, vo_status, device_status

        async def averageData_handler(params):
            time_horizon = 2
            average_temperature = await mean_value(exposed_thing, "temperature", time_horizon)
            average_humidity = await mean_value(exposed_thing, "humidity", time_horizon)
            result = {"average_temperature": average_temperature, "average_humidity": average_humidity}
            print(result)
            return result

        async def check():
            curr_vo_status = await vo_status(exposed_thing, 1)
            curr_device_status = await device_status(exposed_thing, "http://192.168.49.1:9090", 1)

            print(f"Status VO: {curr_vo_status}")
            print(f"Status Device: {curr_device_status}")

        async def read_property_from_device_1():
            temperature = await consumed_vos["device1"].properties["temperature"].read()
            humidity = await consumed_vos["device1"].properties["humidity"].read()
            await exposed_thing.properties["temperature"].write(temperature)
            await exposed_thing.properties["humidity"].write(humidity)


* VO2:

    * Thing Description:

      .. code:: json

        {
            "title": "vo2",
            "id": "urn:dev:wot:plenary:VO2",
            "description": "Lille Plenary Meeting Descriptor for VO2 for Device 2.",
            "securityDefinitions": {
                "no_sc": {
                    "scheme": "nosec"
                }
            },
            "security": "no_sc",
            "@context": [
                "https://www.w3.org/2022/wot/td/v1.1"
            ],
            "properties": {
                "smoke": {
                    "type": "boolean"
                }
            },
            "events": {
                "smokeDetected": {
                    "description": "Smoke Existence from device"
                }
            }
        }


    * VO Descriptor:

      .. code:: yaml

        # ====================================
        # Lille Plenary Meeting VO2 Descriptor
        # ====================================
        name: vo2
        type: VO
        resourceType:
        specification: WoT
        version: 1.1
        bindingNB:
        bindingModeNB: [H,U]
        hostname: vo2
        ports:
            httpPort: 8080
            coapPort: 5683
        securityNB:
            securityScheme: nosec
        deploymentType: B
        catalogue: 9090
        databaseConfig:
        persistentDB:
            SQLite: enabled


    * Python script:

      .. code:: python

        async def smoke_write_handler(value):
            print ("New Smoke Value:", value)
            await exposed_thing._default_update_property_handler("smoke", bool(int(value)))
            if value:
                exposed_thing.emit_event("smokeDetected", "Smoke detected")

* cVO:

    * Thing Description:

      .. code:: json

        {
            "title": "cvo",
            "id": "urn:dev:wot:plenary:cVO",
            "description": "Lille Plenary Meeting Descriptor for cVO.",
            "securityDefinitions": {
                "no_sc": {
                    "scheme": "nosec"
                }
            },
            "security": "no_sc",
            "@context": [
                "https://www.w3.org/2022/wot/td/v1.1"
            ],
            "properties": {
                "temperature": {
                    "type": "integer"
                },
                "humidity": {
                    "type": "integer"
                },
                "smoke": {
                    "type": "boolean"
                },
                "fire": {
                    "type": "boolean"
                }
            },
            "events": {
                "fireDetected": {
                    "description": "Fire Existence Notify APP"
                }
            },
            "actions": {
                "forecast_data": {
                    "description": "Estimate the temperature property with ARIMA"
                }
            }
        }

    * VO Descriptor:

      .. code:: yaml

        # ====================================
        # Lille Plenary Meeting cVO Descriptor
        # ====================================
        name: cvo
        type: cVO
        resourceType:
        specification: WoT
        version: 1.1
        deploymentType: A
        catalogue: 9090
        bindingNB:
        bindingModeNB: [H]
        hostname: cvo
        ports:
            httpPort: 8080
        securityNB:
            securityScheme: nosec
        bindingSB:
        bindingModeSB: [H]
        securitySB:
            securitySBHTTP:
            securityScheme: bearer
            token: token
        databaseConfig:
        timeseriesDB:
            influxDB: enabled
            address: "http://wotpy-influxdb-cvo:8086"
            dbUser: my-username
            dbPass: my-password
            dbToken: my-token
        persistentDB:
            SQLite: enabled
        genericFunction: [forecasting]
        consumedVOs:
        vo1:
            url: "http://vo1:9090/vo1"
        vo2:
            url: "http://vo2:9090/vo2"
            events:
            - smokeDetected
        proxy:
        propertiesMap:
            temperature: vo1
            humidity: vo1


    * Python script:

      .. code:: python

        import asyncio

        from wotpy.functions.functions import forecasting

        fire_init = False

        async def fire_detection(data):
            if "Smoke detected" in str(data):
                smoke = True
            else:
                smoke = False
            await exposed_thing.properties["smoke"].write(smoke)
            results = await consumed_vos["vo1"].invoke_action("currentValues")
            temperature = results["message"]["temperature"]
            if temperature > 30 and smoke:
                exposed_thing.emit_event("fireDetected", "Fire detected")
                await exposed_thing.properties["fire"].write(True)

        def smokeDetected_vo2_on_next(data):
            asyncio.create_task(fire_detection(data))

        def smokeDetected_vo2_on_completed():
            print("Completed")

        def smokeDetected_vo2_on_error(err):
            print(err)

        async def forecast_data_handler(params):
            forecasted_data = await forecasting(exposed_thing, "temperature")
            print ("Forecasted data:", forecasted_data)
            return {'result': True, 'message': {"forecasted_data": forecasted_data}}
