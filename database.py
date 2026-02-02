# database.py
import mysql.connector
from mysql.connector import Error
import datetime
import re

DB_CONFIG = {
    'host': 'localhost',
    'user': 'root',
    'password': '',
    'database': 'integrated',
    'port': 3306
}

class DatabaseHandler:
    def __init__(self):
        self.connection = None
        self.cursor = None
        # Define table prefixes
        self.table_prefixes = {
            'user_header_all': 'UHA',
            'project_worksheet_header_all': 'PWH',
            'road_layers_header_all': 'RLH',
            'road_step_gap_details_all': 'RSG',
            'road_target_header_all': 'RTH',
            'road_target_step_combination_all': 'RTC',
            'worksheet_layers_header_all': 'WLH',
            'work_stage_layers_details_all': 'WSD',
            'measurement_layer_details_all': 'MLD',
            'cross_layers_details_all': 'CLR',
            'project_header_all': 'PRO',
            'bridge_layer_data_details_all': 'BLD'
        }

    def connect(self):
        try:
            self.connection = mysql.connector.connect(**DB_CONFIG)
            self.cursor = self.connection.cursor(dictionary=True)
            return True
        except Error as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        if self.connection and self.connection.is_connected():
            self.cursor.close()
            self.connection.close()

    def execute_query(self, query, params=None, fetch=False):
        try:
            self.cursor.execute(query, params or ())
            if fetch:
                return self.cursor.fetchall()
            self.connection.commit()
            return True
        except Error as e:
            if self.connection:
                self.connection.rollback()
            print(f"Query error: {e}")
            return False

    def get_next_id(self, table_name):
        prefix = self.table_prefixes.get(table_name)
        if not prefix:
            raise ValueError(f"No prefix defined for table: {table_name}")

        query = """
        SELECT * FROM unique_id_header_all 
        WHERE uh_table_name = %s
        """
        self.cursor.execute(query, (table_name,))
        record = self.cursor.fetchone()
        
        now = datetime.datetime.now()
        
        if not record:
            next_id = f"{prefix}-00001"
            insert_query = """
            INSERT INTO unique_id_header_all 
            (uh_table_name, uh_id_for, uh_prefix, uh_last_id, uh_created_on, uh_modified_on)
            VALUES (%s, %s, %s, %s, %s, %s)
            """
            self.execute_query(insert_query, 
                             (table_name, f"{table_name}_id", prefix, next_id, now, now))
            return next_id
        
        last_id = record['uh_last_id']
        
        if not last_id:
            next_id = f"{prefix}-00001"
            update_query = """
            UPDATE unique_id_header_all 
            SET uh_last_id = %s, uh_modified_on = %s
            WHERE uh_id = %s
            """
            self.execute_query(update_query, (next_id, now, record['uh_id']))
            return next_id

        # Parse last_id (UHA-00001, UHA-A0001, etc.)
        last_id_parts = last_id.split('-')
        if len(last_id_parts) != 2:
            raise ValueError(f"Invalid last_id format: {last_id}")

        prefix_part, rest = last_id_parts
        alphabets = ''.join(re.findall(r'[A-Z]', rest))
        digits = ''.join(re.findall(r'\d+', rest))

        alpha_len = len(alphabets)
        digit_len = 5 - alpha_len

        if alpha_len == 5:
            raise ValueError("Reached maximum ID limit: ZZZZZ")

        if digits == '9' * digit_len:
            # Carry over logic (same as before)
            if alphabets == 'Z' and alpha_len == 1:
                alphabets = 'ZA'
                digits = '001'
            elif alphabets == 'ZZ' and alpha_len == 2:
                alphabets = 'ZZA'
                digits = '01'
            elif alphabets == 'ZZZ' and alpha_len == 3:
                alphabets = 'ZZZZ'
                digits = '1'
            elif alphabets == 'ZZZZ' and alpha_len == 4:
                alphabets = 'ZZZZZ'
                digits = ''
            elif alpha_len == 0:
                alphabets = 'A'
                digits = '0001'
            elif alpha_len in [1, 2, 3] and alphabets[-1] != 'Z':
                last_char = alphabets[-1]
                alphabets = alphabets[:-1] + chr(ord(last_char) + 1)
                digits = '1'.zfill(digit_len)
            elif alpha_len in [2, 3] and alphabets[-1] == 'Z':
                alphabets += 'A'
                digits = '1'.zfill(digit_len - 1)
        else:
            next_number = int(digits) + 1
            digits = str(next_number).zfill(digit_len)

        next_id = f"{prefix}-{alphabets}{digits}"
        
        update_query = """
        UPDATE unique_id_header_all 
        SET uh_last_id = %s, uh_modified_on = %s
        WHERE uh_id = %s
        """
        self.execute_query(update_query, (next_id, now, record['uh_id']))
        
        return next_id