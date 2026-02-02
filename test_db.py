from db_config import get_db_connection, close_db_connection, execute_query, execute_update, DB_CONFIG

# Test connection
print("Testing database connection...")
print(f"Host: {DB_CONFIG['host']}")
print(f"Database: {DB_CONFIG['database']}")
print(f"User: {DB_CONFIG['user']}")
print(f"Port: {DB_CONFIG['port']}")
print("-" * 40)
connection = get_db_connection()

if connection:
    print("✓ Connection successful!")
    
    # Test a simple query
    result = execute_query("SELECT 1 as test")
    if result:
        print("✓ Query execution successful!")
        print(f"Result: {result}")
    else:
        print("✗ Query execution failed")
    
    print("-" * 40)
    print("Inserting test data...")
    
    # Data to insert
    add_data = {
        "worksheet_id": "WORK-00003",
        "ph_id": "PRO-00003",
        "user_id": "USER-00003",
        "worksheet_name": "Road Design Cutting",
    }
    
    # Insert query
    insert_query = """
    INSERT INTO project_worksheet_header_all (worksheet_id, ph_id, user_id, worksheet_name)
    VALUES (%(worksheet_id)s, %(ph_id)s, %(user_id)s, %(worksheet_name)s)
    """
    
    if execute_update(insert_query, add_data):
        print("✓ Data inserted successfully!")
        print(f"Inserted data: {add_data}")
    else:
        print("✗ Data insertion failed")
else:
    print("✗ Connection failed")