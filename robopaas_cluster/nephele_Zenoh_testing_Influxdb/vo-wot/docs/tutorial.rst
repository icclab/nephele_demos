.. _tutorial:

Quick Start Guide
=================

Using this repository it is possible to expose Properties, Actions and Events that a device supports.

A full working example is located inside the ``examples/quickstart`` directory of the repository.

.. note::
    This example is a transformed example of the old ``examples/archive/coffe-machine/coffee-machine.py``.
    The new three-file format has been adopted to avoid boilerplate code and to simplify the
    development process.

Setup
-----
This section explains how to use the VO-WoT runtime to expose a device's information.
Three components need to be defined by the user:

* **Thing Description**: a JSON representation of a thing according to the Web of Thing specification
  specifying Properties, Actions and Events.

* **(c)VO descriptor**: a YAML descriptor containing initialization and configuration information.

* **User-defined code**: a Python script where the developer can define the code that will be run
  upon Action invocation, handling of Events etc.

After populating these three files, the user needs to install the python package and execute the cli module as such:

.. code:: bash

    vo-wot(env)$ pip install vo-wot
    vo-wot(env)$ vo-wot -t <thing_description>.json -f <vo_descriptor>.yaml <script_file>.py

Thing Description
-----------------

The main element that contains all the logic of the device to be exposed
is the Thing Description. All the support keywords are explained
`here <https://www.w3.org/TR/wot-thing-description11/>`__. The
Thing Description is a JSON representation containing metadata and the
key components are the properties, the actions and the events. A
playground is available to create and validate Thing Descriptions (TDs)
`here <http://plugfest.thingweb.io/playground/>`__.

.. warning::
    In contrast to the Web of Things specification mentioned above, forms are not
    written by the developer. They are auto-generated instead.

Thing Description basics
~~~~~~~~~~~~~~~~~~~~~~~~

A Thing Description is a JSON document. The mandatory elements are the following:

=========================  ==================================================
Field                      Description
=========================  ==================================================
``@context``               refers to the Thing Description specification used
``title``                  the title of the Thing Description/Device
``security``               refers to the security scheme that will be used to gain access to the servient from a client and must be one of the security schemes defined inside the ``securityDefinitions``
``securityDefinitions``    list of possible types of security that will be used along with their scheme and other necessary information
=========================  ==================================================

Therefore a barebones Thing Description can be the following:

.. code:: json

    {
        "title": "Smart-Coffee-Machine",
        "@context": [
            "https://www.w3.org/2022/wot/td/v1.1"
        ],
        "securityDefinitions": {
            "nosec_sc": {
                "scheme": "nosec"
            }
        },
        "security": "nosec_sc"
    }


Properties
~~~~~~~~~~

The properties refer to the parameters or state of a device.

Examples of simple properties are:

.. code:: json

    "properties": {
        "allAvailableResources": {
            "type": "object",
            "description": "Current level of all available resources given as an integer percentage for each particular resource.The data is obtained from the machine's sensors but can be set manually in case the sensors are broken.",
            "properties": {
                "water": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 100
                },
                "milk": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 100
                },
                "chocolate": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 100
                },
                "coffeeBeans": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 100
                }
            }
        },
        "servedCounter": {
            "type": "integer",
            "description": "The total number of served beverages.",
            "minimum": 0
        },
        "maintenanceNeeded": {
            "type": "boolean",
            "description": "Shows whether a maintenance is needed. The property is observable. Automatically set to True when the servedCounter property exceeds 1000.",
            "observable": true
        }
    }

Actions
~~~~~~~

The actions refer to physical processes related to the device and offer a certain functionality. Examples are:

.. code:: json

    "actions": {
        "makeDrink": {
            "description": "Make a drink from available list of beverages. Accepts drink id, size and quantity as input. Brews one medium americano if no input is specified.",
            "input": {
                "type": "object",
                "properties": {
                    "drinkId": {
                        "type": "string",
                        "description": "Defines what drink to make, drinkId is one of possibleDrinks property values, e.g. latte."
                    },
                    "size": {
                        "type": "string",
                        "description": "Defines the size of a drink, s = small, m = medium, l = large.",
                        "enum": ["s", "m", "l"]
                    },
                    "quantity": {
                        "type": "integer",
                        "description": "Defines how many drinks to make, ranging from 1 to 5.",
                        "minimum": 1,
                        "maximum": 5
                    }
                }
            },
            "output": {
                "type": "object",
                "description": "Returns True/false and a message when all invoked promises are resolved (asynchronous).",
                "properties": {
                    "result": {
                        "type": "boolean"
                    },
                    "message": {
                        "type": "string"
                    }
                }
            }
        }
    }

Events
~~~~~~

The events are used for the push model of communication where notifications, discrete events or streams of values are sent asynchronously to the receiver. Examples are:

.. code:: json

    "events": {
        "outOfResource": {
            "description": "Out of resource event. Emitted when the available resource level is not sufficient for a desired drink.",
            "data": {
                "type": "string"
            }
        }
    }

Python script
-------------

The python script is where the user needs to define all the code regarding setting property values,
invoking actions or handling events.

Servient initialization
~~~~~~~~~~~~~~~~~~~~~~~

A Servient is a software stack that implements the WoT building blocks. It is used under the hood
to both expose Things (server) and to consume them (client). The Servient starts up an HTTP server
by convention (Catalogue server) on the default port 9090. From there on depending on which protocols
have been enabled for the NorthBound interfaces in the Virtual Object Descriptors, the corresponding
protocol servers are started. Lastly, the Thing Description is parsed and is then ready to be served
on the Catalogue server port.

Exposed Thing API
~~~~~~~~~~~~~~~~~

The ``exposed_thing`` object is an abstraction of the Virtual Object and allows configuring the Virtual
Object's behavior. This object is injected into the user-defined code inside of the python
script. It is created when the Virtual Object starts serving its data and can be used by the developer to
execute functions of the WoT scripting API such as writing and reading Properties, invoking Actions and
emitting Events. Some examples are:

.. code:: py

    await exposed_thing.write_property(
        'maintenanceNeeded',
        False
    )
    value = await exposed_thing.read_property(
        'maintenanceNeeded'
    )
    makeCoffee = await exposed_thing.invoke_action(
        'makeDrink',
        {'drinkId': 'latte', 'size': 'l', 'quantity': 3}
    )
    exposed_thing.emit_event(
        'outOfResource',
        f'Low level of {resource}: {resources[resource]}%'
    )


Virtual Object consumption
~~~~~~~~~~~~~~~~~~~~~~~~~~

Depending on which SouthBound protocols have been enabled in the Virtual Object Descriptor,
the corresponding protocol clients will be created under the hood. Afterwards, in the
``consumedVOs`` section of the descriptor the developer can specify which foreign
Virtual Objects will be consumed. After these virtual objects are consumed, a ``consumed_thing``
object is created. These objects are then stored in a dictionary called ``consumed_vos`` where
the keys are the name of each consumed Virtual Object (specified in the Virtual Object Descriptor)
and the value is the consumed thing object.

.. note::
    The **Exposed Thing** abstracts the locally created Virtual Object while the **Consumed Thing**
    abstracts a remote Virtual Object whose data we want to consume.

Consumed Thing API
~~~~~~~~~~~~~~~~~~

The ``consumed_vos`` dictionary as mentioned above is injected into the user-defined code inside of the python
script. It can  be used to read, write properties and invoke actions. For example:

.. code:: py

    allAvailableResources = await consumed_vos["vo1"].read_property(
        'allAvailableResources'
    )

    allAvailableResources['water'] = 80
    await consumed_vos["vo1"].write_property(
        'allAvailableResources',
        allAvailableResources
    )

    makeCoffee = await consumed_vos["vo1"].invoke_action(
        'makeDrink',
        {'drinkId': 'latte', 'size': 'l', 'quantity': 3}
    )


Populating the python script
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To ease the development of a Virtual Object, several assumptions are made to map
constructs defined inside the Thing Description to user-defined snippets of code.

As mentioned above the following variables are injected to the user-defined code inside the python script:

* ``exposed_thing``: An object to interact with the locally Exposed Thing/Virtual Object.

* ``consumed_vos``: A dictionary mapping remote Virtual Object names to Consumed Things/Virtual Objects.

Properties
""""""""""

For each property, the developer can set an initial value, set the read/write handlers to specify what
operation needs to be performed each time the property is read/written to and set the functions used to
subscribe to local property changes:

* **Initial value**: In the user-defined code the developer needs to set the value of a special variable named
  ``<property_name>_init``. For example, the ``maintenanceNeeded`` property can initially be set to False
  as such:

    .. code:: python

        maintenanceNeeded_init = False

* **Read handler**: In the user-defined code the developer can **optionally** set what operation will be performed
  to fetch the property value by defining an asynchronous function with the name ``<property_name>_read_handler``.
  This is useful if the property value is stored elsewhere and needs to be fetched
  or if extra code needs to run before/after the default property read handler. The default read handler
  simple fetches the value stored locally and can be invoked by calling the function
  ``exposed_thing._default_retrieve_property_handler(propertyName)`` where ``propertyName`` is a string
  with the property name.

    .. code:: python

        async def maintenanceNeeded_read_handler():
            value = exposed_thing._default_retrieve_property_handler("maintenanceNeeded")
            if value is True:
                print("Maintenance needed")

* **Write handler**: In the user-defined code the developer can **optionally** set what operation will be performed
  to write to the property value by defining an asynchronous function with the name ``<property_name>_write_handler``.
  This is useful if extra code needs to run before/after the default property write handler. The default write handler
  simple stores the value locally and can be invoked by calling the function
  ``exposed_thing._default_update_property_handler(propertyName)`` where ``propertyName`` is a string
  with the property name.

    .. code:: python

        async def servedCounter_write_handler(value):
            await exposed_thing._default_update_property_handler('servedCounter', value)

            if value > 1000:
                await exposed_thing.write_property('maintenanceNeeded', True)

* **Property change subscriptions**: In the user-defined code the user can **optionally** subscribe to local
  property changes and specify three functions: the ``on_next``, the ``on_completed`` and the ``on_error``
  functions that get called each time the property changes, each time the subscription for some reason
  finishes and when an error occurs respectively. Only the ``on_next`` is mandatory to subscribe to a
  property change. The other two functions do not have to be explicitly set. The function names have
  to be set to ``<property_name>_on_next``, ``<property_name>_on_completed`` and ``<property_name>_on_error``.

    .. code:: python

        def maintenanceNeeded_on_next(data):
            print(f'Value changed for an observable property: {data}')

        def maintenanceNeeded_on_completed():
            print('Subscribed for an observable property: maintenanceNeeded')

        def maintenanceNeeded_on_error(error):
            print(f'Error for an observable property maintenanceNeeded: {error}')

Actions
"""""""

For each action, the developer can set an action handler to specify the function that will be run
each time the action is invoked:

* **Action invocation handler**: In the user-defined code the developer **must** set what operation will be performed
  each time the action is invoked by defining an asynchronous function with the name ``<action_name>_handler``
  that receives as input the parameter ``params`` with the action input.

    .. code:: python

        async def setSchedule_handler(params):
        params = params['input'] if params['input'] else {}

        # Check if required fields are provided in input
        if 'time' in params and 'mode' in params:

            # Use default values for non-required fields if not provided
            params['drinkId'] = params.get('drinkId', 'americano')
            params['size'] = params.get('size', 'm')
            params['quantity'] = params.get('quantity', 1)

            # Now read the schedules property, add a new schedule to it and then rewrite the schedules property
            schedules = await exposed_thing.read_property('schedules')
            schedules.append(params)
            await exposed_thing.write_property('schedules', schedules)
            return {'result': True, 'message': 'Your schedule has been set!'}

        return {'result': False, 'message': 'Please provide all the required parameters: time and mode.'}


Events
""""""

Events can **optionally** be locally subscribed to so that certain snippets of code run each time an event
is emitted.

* **Event subscriptions**: In the user-defined code the user can **optionally** subscribe to locally
  emitted events and specify three functions: the ``on_next``, the ``on_completed`` and the ``on_error``
  functions that get called each time a specific event is emitted, each time the subscription for some reason
  finishes and when an error occurs respectively. Only the ``on_next`` is mandatory to subscribe to an
  event. The other two functions do not have to be explicitly set. The function names have
  to be set to ``<event_name>_on_next``, ``<event_name>_on_completed`` and ``<event_name>_on_error``.

.. code:: python

    def outOfResource_on_next(data):
        print(f'Out of resource event emitted. Data: {data}')

    def outOfResource_on_completed():
        print('Subscription completed')

    def outOfResource_on_error(error):
        print(f'Error for an event outOfResource: {error}')


Summary
~~~~~~~

Properties
""""""""""

* Set property initial value
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            maintenanceNeeded_init = False

* Set property read handler
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            async def maintenanceNeeded_read_handler():
                value = exposed_thing._default_retrieve_property_handler("maintenanceNeeded")
                if value is True:
                    print("Maintenance needed")

* Set property write handler
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            async def servedCounter_write_handler(value):
                await exposed_thing._default_update_property_handler('servedCounter', value)

                if value > 1000:
                    await exposed_thing.write_property('maintenanceNeeded', True)

* Read property value
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            value = await exposed_thing.read_property(
                'maintenanceNeeded'
            )

* Write property value
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            await exposed_thing.write_property(
                'maintenanceNeeded',
                False
            )

* Subscribe to local property change
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            def outOfResource_on_next(data):
                print(f'Out of resource event emitted. Data: {data}')

            def outOfResource_on_completed():
                print('Subscription completed')

            def outOfResource_on_error(error):
                print(f'Error for an event outOfResource: {error}')

* Subscribe to remote property change
    - **Declared in:** Virtual Object Descriptor, Python script
    - **Example:**

        - Virtual Object Descriptor:

        .. code:: yaml

            consumedVOs:
                vo1:
                    propertyChanges:
                    - temperature

        - Python script:

        .. code:: python

            def temperature_vo1_on_next(data):
                print(data)

            def smokeDetectetemperature_vo1_on_completed():
                print("Completed")

            def temperature_vo1_on_error(err):
                print(err)

Actions
"""""""

* Set action invocation handler
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            async def setSchedule_handler(params):
                params = params['input'] if params['input'] else {}

                # Check if required fields are provided in input
                if 'time' in params and 'mode' in params:

                    # Use default values for non-required fields if not provided
                    params['drinkId'] = params.get('drinkId', 'americano')
                    params['size'] = params.get('size', 'm')
                    params['quantity'] = params.get('quantity', 1)

                    # Now read the schedules property, add a new schedule to it and then rewrite the schedules property
                    schedules = await exposed_thing.read_property('schedules')
                    schedules.append(params)
                    await exposed_thing.write_property('schedules', schedules)
                    return {'result': True, 'message': 'Your schedule has been set!'}

                return {'result': False, 'message': 'Please provide all the required parameters: time and mode.'}

* Invoke action
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            makeCoffee = await consumed_vos["vo1"].invoke_action(
                'makeDrink',
                {'drinkId': 'latte', 'size': 'l', 'quantity': 3}
            )

Events
""""""

* Emit event
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            exposed_thing.emit_event(
                'outOfResource',
                'Out of resources'
            )

* Subscribe to local event
    - **Declared in:** Python script
    - **Example:**

        .. code:: python

            def outOfResource_on_next(data):
                print(f'Out of resource event emitted. Data: {data}')

            def outOfResource_on_completed():
                print('Subscription completed')

            def outOfResource_on_error(error):
                print(f'Error for an event outOfResource: {error}')

* Subscribe to remote event
    - **Declared in:** Virtual Object Descriptor, Python script
    - **Example:**

        - Virtual Object Descriptor:

        .. code:: yaml

            consumedVOs:
                vo1:
                    events:
                    - outOfResource

        - Python script:

        .. code:: python

            def outOfResource_vo1_on_next(data):
                print(f'Out of resource event emitted. Data: {data}')

            def outOfResource_vo1_on_completed():
                print('Subscription completed')

            def outOfResource_vo1_on_error(error):
                print(f'Error for an event outOfResource: {error}')

The exact structure of the Virtual Object Descriptor is described in the next section :ref:`voDescriptor`.