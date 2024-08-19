Consuming VO Data
===================

To consume data from the Virtual Object according to the Web of Things specification,
the developer needs to use the Consumed Thing API.

Each Virtual Object is described by its Thing Description (TD) containing information
on how to interact with it. As such, the first step in communicating with the VO
is the consumption of the Thing Description:

.. code:: python

    from wotpy.wot.servient import Servient
    from wotpy.wot.wot import WoT

    wot = WoT(servient=Servient())
    vo_url = 'http://127.0.0.1:9090/smart-coffee-machine'
    consumed_thing = await wot.consume_from_url(vo_url)

Afterwards, the ``consumed_thing`` variable serves as an abstraction of the
remote Virtual Object and the interaction with the remote VO happens through
the Consumed Thing API. Clients of the protocols are created implicitly and
are used to communicate with the VO through the forms described in the
Thing Description.

Read property
~~~~~~~~~~~~~

.. code:: python

    property_value = await consumed_thing.read_property('allAvailableResources')

The above code will read the value of the property ``allAvailableResources``.

Write property
~~~~~~~~~~~~~~

.. code:: python

    await consumed_thing.write_property('allAvailableResources', allAvailableResources)

The above code will write the value of the property ``allAvailableResources``.


Subscribe to property changes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: python

    consumed_thing.properties['maintenanceNeeded'].subscribe(
        on_next=lambda data: LOGGER.info(f'Value changed for an observable property: {data}'),
        on_completed=LOGGER.info('Subscribed for an observable property: maintenanceNeeded'),
        on_error=lambda error: LOGGER.info(f'Error for an observable property maintenanceNeeded: {error}')
    )

The above code will subscribe to property changes of the property ``maintenanceNeeded`` and defines three functions
for what happens when there are new data (on_next), when the subscription completes (on_complete) and
what happens when an error occurs (on_error).


Invoke an action
~~~~~~~~~~~~~~~~

.. code:: python

    makeCoffee = await consumed_thing.invoke_action('makeDrink', {'drinkId': 'latte', 'size': 'l', 'quantity': 3})
    if makeCoffee.get('result'):
        LOGGER.info('Enjoy your drink! \n{}'.format(makeCoffee))
    else:
        LOGGER.info('Failed making your drink: {}'.format(makeCoffee))

The above code invokes the action ``makeDrink`` and sends as input the value ``{'drinkId': 'latte', 'size': 'l', 'quantity': 3}``.
The return value, if set, is fetched through the return value of the action invocation.


Subscribe to events
~~~~~~~~~~~~~~~~~~~

Similarly to property changes events can be subscribed to:

.. code:: python

    consumed_thing.events['outOfResource'].subscribe(
        on_next=lambda data: LOGGER.info(f'New event is emitted: {data}'),
        on_completed=LOGGER.info('Subscribed for an event: outOfResource'),
        on_error=lambda error: LOGGER.info(f'Error for an event outOfResource: {error}')
    )

The above code will subscribe to emitted events of the event ``maintenanceNeeded`` and defines three functions
for what happens when there are new data (on_next), when the subscription completes (on_complete) and
what happens when an error occurs (on_error).
