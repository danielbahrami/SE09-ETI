import json
import sys
from io import BytesIO

from mcap.reader import make_reader


def mcap_to_json(file_name):
    messages = []
    with open(file_name, 'rb') as file:
        b = file.read()
        with BytesIO(b) as f:
            reader = make_reader(f)
            for message in reader.iter_messages():
                message_data = {
                    'channel': message[2].channel_id,
                    'log_time': message[2].log_time,
                    'topic': message[1].topic,
                    'encoding': message[1].message_encoding,
                    'data': message[2].data.decode('utf-8', errors='ignore')
                }
                messages.append(message_data)
    json_string = json.dumps(messages)
    return json_string


def main():
    sys.stdout.write(mcap_to_json(sys.argv[1]))


"""     with open('dummy/data/joint_states/joint_states_0.mcap', 'rb') as f:
        b = f.read()
        json_string = mcap_to_json(b)
        #print(json_string) """

if __name__ == '__main__':
    main()
