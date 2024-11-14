CREATE TABLE ros_bags (
    id SERIAL PRIMARY KEY,
    robot_name VARCHAR(100),
    received_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    robot_sent_at TIMESTAMP,
    ros_bag_data BYTEA
);
