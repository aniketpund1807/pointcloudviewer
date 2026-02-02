import mysql.connector
from mysql.connector import Error

# Local database configuration
DB_CONFIG = {
    'host': 'localhost',
    'user': 'root',
    'password': '',
    'database': 'integrated',
    'port': 3306
}


# # Live server database configuration
# DB_CONFIG = {
#     'host': '103.212.120.166 ',
#     'user': 'injlgsty_integrated',
#     'password': 'j_2WDs}m9Uc~Jr?T',
#     'database': 'injlgsty_integrated',
#     'port': 3306
# }


def get_db_connection():
    """
    Create and return a MySQL database connection
    """
    try:
        connection = mysql.connector.connect(**DB_CONFIG)
        if connection.is_connected():
            db_info = connection.get_server_info()
            print(f"Successfully connected to MySQL Server version {db_info}")
            return connection
    except Error as e:
        if e.errno == 2003:
            print(f"Error: Unable to connect to MySQL Server on '{DB_CONFIG['host']}'")
        elif e.errno == 1045:
            print("Error: Access denied (check username/password)")
        elif e.errno == 1049:
            print(f"Error: Unknown database '{DB_CONFIG['database']}'")
        else:
            print(f"Error while connecting to MySQL: {e}")
        return None

def close_db_connection(connection):
    """
    Close the database connection
    """
    if connection and connection.is_connected():
        connection.close()
        print("MySQL connection is closed")

def execute_query(query, params=None):
    """
    Execute a SELECT query and return results
    """
    connection = get_db_connection()
    if connection is None:
        return None
    
    try:
        cursor = connection.cursor(dictionary=True)
        if params:
            cursor.execute(query, params)
        else:
            cursor.execute(query)
        result = cursor.fetchall()
        cursor.close()
        return result
    except Error as e:
        print(f"Error executing query: {e}")
        return None
    finally:
        close_db_connection(connection)

def execute_update(query, params=None):
    """
    Execute an INSERT, UPDATE, or DELETE query
    """
    connection = get_db_connection()
    if connection is None:
        return False
    
    try:
        cursor = connection.cursor()
        if params:
            cursor.execute(query, params)
        else:
            cursor.execute(query)
        connection.commit()
        print(f"Query executed successfully. Rows affected: {cursor.rowcount}")
        cursor.close()
        return True
    except Error as e:
        print(f"Error executing update: {e}")
        connection.rollback()
        return False
    finally:
        close_db_connection(connection)
