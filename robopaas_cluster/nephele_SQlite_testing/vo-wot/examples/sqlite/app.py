import logging

LOGGER = logging.getLogger()

TABLE_NAME = "string_data_table"


async def someStringProperty_write_handler(value):
    servient = exposed_thing.servient
    sqlite_db = servient.sqlite_db
    name = value["name"]
    content = value["content"]

    columns = {
        "name": "TEXT",
        "content": "TEXT"
    }
    sqlite_db.create_table_if_not_exists(TABLE_NAME, columns)
    sqlite_db.insert_data(TABLE_NAME, (name, content))


async def someStringProperty_read_handler():
    servient = exposed_thing.servient
    sqlite_db = servient.sqlite_db

    return servient.sqlite_db.fetch_all_rows(TABLE_NAME)
