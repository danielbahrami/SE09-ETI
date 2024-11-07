import requests

def upload_file_to_db(file):

    url = "http://<my-api>:<5000>/api "  # Replace with target API URL

    headers = {
        'Content-Type': 'application/octet-stream',
    }

    with open(file, 'rb') as file:
        data = file.read()
    
    response = requests.post(url=url, data=data, headers=headers)

    # Check if the request was successful
    if response.status_code == 200:
        return True
    else:
        return False
