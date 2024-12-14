import requests
import os
import pwd
import datetime

def upload_file_to_db(topic_name, file):

    url = "http://<my-api>:<5000>/api "  # Replace with target API URL

    headers = {'Content-Type': 'application/octet-stream',}

    with open(file, 'rb') as file:
        data = file.read()
    
    payload = {'data': data, 'Robot_user': pwd.getpwuid(os.getuid())[0], 'Date': datetime.datetime.now(), 'topic': topic_name}

    response = requests.post(url=url, data=payload, headers=headers)

    # Check if the request was successful
    if response.status_code == 200:
        return True
    else:
        return False