#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Class that handles sqlite database operations.
"""

import sqlite3

from wotpy.database.database_schema import DB_SCHEMA


class SQLiteDatabase:
    """Class that handles all sqlite database operations"""

    def __init__(self, db_path):
        self.db_path = db_path if db_path is not None else "vo.db"
        self.conn = sqlite3.connect(self.db_path)
        self.conn.executescript(DB_SCHEMA)
        self.conn.commit()

    def execute_query(self, query):
        """Execute the provided SQL query on the database and return the result"""

        cursor = self.conn.cursor()
        cursor.execute(query)
        result = cursor.fetchall()
        cursor.close()
        return result

    def fetch_all_rows(self, table_name):
        """Fetches all rows of the given table"""

        if not table_name.isidentifier():
            raise ValueError(f"Invalid table name: {table_name}")

        cursor = self.conn.cursor()

        query = f"SELECT * FROM {table_name}"
        cursor.execute(query)
        rows = cursor.fetchall()

        cursor.close()

        return rows

    def insert_data(self, table_name, data):
        """Insert the provided data into the specified table"""

        if not table_name.isidentifier():
            raise ValueError("Invalid table name")

        cursor = self.conn.cursor()
        placeholders = ",".join(["?" for i in range(len(data))])
        query = f"INSERT OR REPLACE INTO {table_name} VALUES ({placeholders})"
        cursor.execute(query, data)
        self.conn.commit()
        cursor.close()

    def create_table_if_not_exists(self, table_name, columns):
        """Create a table if it doesn't exist."""

        cursor = self.conn.cursor()

        if not table_name.isidentifier():
            raise ValueError("Invalid table name")

        for column_name, column_type in columns.items():
            if not column_name.isidentifier():
                raise ValueError(f"Invalid column name: {column_name}")
            if not column_type.upper() in {"INTEGER", "TEXT", "REAL", "BLOB", "NUMERIC", "INTEGER PRIMARY KEY"}:
                raise ValueError(f"Invalid data type for column {column_name}: {column_type}")

        columns_def = ", ".join([f"{name} {dtype}" for name, dtype in columns.items()])
        query = f"CREATE TABLE IF NOT EXISTS {table_name} ({columns_def})"

        cursor.execute(query)
        self.conn.commit()
        cursor.close()
