#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Class that defines the base database schema.
"""

DB_SCHEMA = """
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
"""