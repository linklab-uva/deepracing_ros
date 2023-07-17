from typing import List, Tuple, Union
import numpy as np
import os
import tqdm
import yaml
import multiprocessing, multiprocessing.pool

from rclpy.serialization import deserialize_message 
from rosidl_runtime_py.utilities import get_message
import rclpy
import rosbag2_py

import deepracing_msgs.msg

def get_rosbag_options(path, serialization_format="cdr", storage_id="sqlite3"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

def open_bagfile(filepath: str, serialization_format="cdr"):
    if not os.path.isdir(filepath):
        raise ValueError("%s is not a directory" % (filepath,))
    
    metadatafilepath = os.path.join(filepath, "metadata.yaml")
    if not os.path.isfile(metadatafilepath):
        raise ValueError("%s does not contain a file named metadata.yaml" % (filepath,))
    
    with open(metadatafilepath, "r") as f:
        metadata : dict = yaml.load(f, Loader=yaml.SafeLoader)["rosbag2_bagfile_information"]

    storage_id : str = metadata["storage_identifier"]
    storage_options, converter_options = get_rosbag_options(filepath, serialization_format=serialization_format, storage_id=storage_id)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    # Create maps for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    topic_metadata_map = {topic_types[i].name: topic_types[i] for i in range(len(topic_types))}

    return topic_types, type_map, topic_metadata_map, reader

def open_bagfile_writer(filepath: str, serialization_format="cdr", storage_id="sqlite3"):
    storage_options, converter_options = get_rosbag_options(filepath, serialization_format=serialization_format, storage_id=storage_id)

    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    return writer


def systemTimeKey( msg : Union[deepracing_msgs.msg.TimestampedPacketSessionData, deepracing_msgs.msg.TimestampedPacketLapData, deepracing_msgs.msg.TimestampedPacketMotionData] ):
    return rclpy.time.Time.from_msg(msg.header.stamp)

def sessionTimeKey( msg : Union[deepracing_msgs.msg.TimestampedPacketSessionData, deepracing_msgs.msg.TimestampedPacketLapData, deepracing_msgs.msg.TimestampedPacketMotionData] ):
    return msg.udp_packet.header.session_time

def allFromTopic(bag_dir : str, topicname : str) -> list:
    metadatafile : str = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.isfile(metadatafile):
        raise ValueError("Metadata file %s does not exist. Are you sure %s is a valid rosbag?" % (metadatafile, bag_dir))
    with open(metadatafile, "r") as f:
        metadata_dict : dict = yaml.load(f, Loader=yaml.SafeLoader)["rosbag2_bagfile_information"]
    topic_count_dict = {entry["topic_metadata"]["name"] : entry["message_count"] for entry in metadata_dict["topics_with_message_count"]}
    _, type_map, _, reader = open_bagfile(bag_dir)
    msg_type = type_map[topicname]
    msg_type_full = get_message(msg_type)
    filt = rosbag2_py.StorageFilter([topicname])
    reader.set_filter(filt)
    out : list = []
    for idx in tqdm.tqdm(iterable=range(topic_count_dict[topicname]), desc="Loading data from topic %s in bag: %s" % (topicname, bag_dir)):
        if(reader.has_next()):
            (topic, data, t) = reader.read_next()
        out.append(deserialize_message(data, msg_type_full))
    return out

def getAllData(bag_dir : str)\
      -> Tuple[List[deepracing_msgs.msg.TimestampedPacketMotionData],\
               List[deepracing_msgs.msg.TimestampedPacketLapData],\
               List[deepracing_msgs.msg.TimestampedPacketSessionData]]:
    metadatafile : str = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.isfile(metadatafile):
        raise ValueError("Metadata file %s does not exist. Are you sure %s is a valid rosbag?" % (metadatafile, bag_dir))
    
    motiondata_topic : str = "/motion_data"
    lapdata_topic : str = "/lap_data"
    sessiondata_topic : str = "/session_data"

    # threadpool : multiprocessing.pool.Pool = multiprocessing.pool.Pool(processes=3)
    # motion_result : multiprocessing.pool.AsyncResult = threadpool.apply_async(allFromTopic, (bag_dir, motiondata_topic)) 
    # lap_result : multiprocessing.pool.AsyncResult = threadpool.apply_async(allFromTopic, (bag_dir, lapdata_topic)) 
    # session_result : multiprocessing.pool.AsyncResult = threadpool.apply_async(allFromTopic, (bag_dir, sessiondata_topic)) 
    # threadpool.close()
    # threadpool.join()

    # motion_packets : List[deepracing_msgs.msg.TimestampedPacketMotionData] = motion_result.get()
    # lap_packets : List[deepracing_msgs.msg.TimestampedPacketLapData] = lap_result.get()
    # session_packets : List[deepracing_msgs.msg.TimestampedPacketSessionData] = session_result.get()

    with open(metadatafile, "r") as f:
        metadata_dict : dict = yaml.load(f, Loader=yaml.SafeLoader)["rosbag2_bagfile_information"]
    topic_count_dict = {entry["topic_metadata"]["name"] : entry["message_count"] for entry in metadata_dict["topics_with_message_count"]}
    _, _, _, reader = open_bagfile(bag_dir)


    topics = [lapdata_topic, motiondata_topic, sessiondata_topic]
    count = int(np.sum(np.asarray([topic_count_dict[t] for t in topics], dtype=np.int64)))
    filt = rosbag2_py.StorageFilter(topics)
    reader.set_filter(filt)

    motiondata_type = get_message("deepracing_msgs/msg/TimestampedPacketMotionData")
    lapdata_type = get_message("deepracing_msgs/msg/TimestampedPacketLapData")
    session_type = get_message("deepracing_msgs/msg/TimestampedPacketSessionData")

    motion_packets : List[deepracing_msgs.msg.TimestampedPacketMotionData] = []
    lap_packets : List[deepracing_msgs.msg.TimestampedPacketLapData] = []
    session_packets : List[deepracing_msgs.msg.TimestampedPacketSessionData] = []

    for idx in tqdm.tqdm(iterable=range(count), desc="Loading data from bag: %s" % (bag_dir,)):
        if(reader.has_next()):
            (topic, data, t) = reader.read_next()
        else:
            break
        if topic==motiondata_topic:
            current_msg : deepracing_msgs.msg.TimestampedPacketMotionData = deserialize_message(data, motiondata_type)
            motion_packets.append(current_msg)
        elif topic==lapdata_topic:
            current_msg : deepracing_msgs.msg.TimestampedPacketLapData = deserialize_message(data, lapdata_type)
            lap_packets.append(current_msg)
        elif topic==sessiondata_topic:
            current_msg : deepracing_msgs.msg.TimestampedPacketSessionData = deserialize_message(data, session_type)
            session_packets.append(current_msg)
        
    motion_packets_sorted = sorted(motion_packets, key=sessionTimeKey)
    isearch_motion = 0
    while motion_packets_sorted[isearch_motion].udp_packet.header.session_time<=0.0:
        isearch_motion+=1
    if isearch_motion>0:
        isearch_motion-=1

    lap_packets_sorted = sorted(lap_packets, key=sessionTimeKey)
    isearch_lap = 0
    while lap_packets_sorted[isearch_lap].udp_packet.header.session_time<=0.0:
        isearch_lap+=1
    if isearch_lap>0:
        isearch_lap-=1


    session_packets_sorted = sorted(session_packets, key=sessionTimeKey)
    isearch_session = 0
    while session_packets_sorted[isearch_session].udp_packet.header.session_time<=0.0:
        isearch_session+=1
    if isearch_session>0:
        isearch_session-=1


    return motion_packets_sorted[isearch_motion:], lap_packets_sorted[isearch_lap:], session_packets_sorted[isearch_session:]