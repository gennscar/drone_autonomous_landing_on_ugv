import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import csv


class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        # create a message type map
        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of,
                           name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of,
                         name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: get_message(
            type_of) for id_of, name_of, type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


if __name__ == "__main__":
    bag_file = 'rosbags/FixedTag0/FixedTag0_0.db3'

    parser = BagFileParser(bag_file)

    data = parser.get_messages("/uwb_ranging")

    form = []
    form.append(['timestamp', 'range_ok', 'range_mes', 'first_peak_pwr',
                 'rx_pwr', 'cir_layout', 'cir', 'max_noise', 'std_noise'])
    for row in data:
        form.append([row[0], list(row[1].range_ok), list(row[1].range_mes), list(row[1].first_peak_pwr), list(row[1].rx_pwr),
                    list(row[1].cir_layout), list(row[1].cir), list(row[1].max_noise), list(row[1].std_noise)])

    with open(bag_file + '.csv', 'w', newline='') as file:
        writer = csv.writer(file, delimiter=';')
        writer.writerows(form)
