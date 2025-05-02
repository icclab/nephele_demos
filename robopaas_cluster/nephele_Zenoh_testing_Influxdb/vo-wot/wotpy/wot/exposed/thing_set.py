#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Class that represents a group or set of ExposedThing instances that exist in the same context.
"""


class ExposedThingSet:
    """Represents a group of ExposedThing objects.
    A group cannot contain two ExposedThing with the same Thing ID."""

    def __init__(self):
        self._exposed_things = {}

    @property
    def exposed_things(self):
        """A generator that yields all the ExposedThing contained in this group."""

        for exposed_thing in self._exposed_things.values():
            yield exposed_thing

    def contains(self, exposed_thing):
        """Returns True if this group contains the given ExposedThing."""

        return exposed_thing in self._exposed_things.values()

    def add(self, exposed_thing):
        """Add a new ExposedThing to this set."""

        if exposed_thing.thing.title in self._exposed_things:
            raise ValueError("Duplicate Exposed Thing: {}".format(exposed_thing.title))

        self._exposed_things[exposed_thing.thing.title] = exposed_thing

    def remove(self, thing_name):
        """Removes an existing ExposedThing by Name."""

        exposed_thing = self.find_by_thing_name(thing_name)

        if exposed_thing is None:
            raise ValueError("Unknown Exposed Thing: {}".format(thing_name))

        assert exposed_thing.thing.title in self._exposed_things
        self._exposed_things.pop(exposed_thing.thing.title)

    def find_by_thing_name(self, thing_name):
        """Finds an existing ExposedThing by Thing Name."""

        def is_match(exp_thing):
            return exp_thing.thing.title == thing_name or exp_thing.thing.url_name == thing_name

        return next((item for item in self._exposed_things.values() if is_match(item)), None)

    def find_by_interaction(self, interaction):
        """Finds the ExposedThing whose Thing contains the given Interaction."""

        def is_match(exp_thing):
            return exp_thing.thing is interaction.thing

        return next((item for item in self._exposed_things.values() if is_match(item)), None)
