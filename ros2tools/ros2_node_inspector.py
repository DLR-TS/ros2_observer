#!/usr/bin/env python3

import sys
import json
import os
import subprocess
import re
import argparse
import copy


from ros2tools import ROS2Tools
try:
    from ros2_tools.util import *
    from ros2_tools.trace_converter import *
    from ros2_tools.ros2_node_grapher import *
except ImportError:
    sys.path.append(os.path.join(os.path.dirname(__file__), "."))
    from util import *
    from trace_converter import *
    from ros2_node_grapher import *


NODES_JSON_FILE = "nodes.json"
DATATYPES_JSON_FILE = "datatypes.json"
GRAPH_JSON_FILE = "graph.json"
NODE_SUMMARIES_JSON_FILE = "node_summaries.json"
GRAPH_FILE = "graph.html"

ros_home = os.getenv('ROS_HOME', os.path.expanduser('~/.ros'))
os.environ['ROS_HOME'] = ros_home
OUTPUT_DIRECTORY = os.path.join(os.environ['ROS_HOME'], 'ros2_node_inspector')

try:
    check_command_installed("ros2")
except:
    raise Exception("ERROR: ROS2 is not installed or not found in the system path. Install it and try again.")


def delete_key_recursive(data, target_key):
    if isinstance(data, dict):
        for key in [k for k in data if k == target_key]:
            del data[key]
        for value in data.values():
            delete_key_recursive(value, target_key)
    elif isinstance(data, list):
        for item in data:
            delete_key_recursive(item, target_key)
    return data


def sanitize_filename(node_name):
    return re.sub(r'[^a-zA-Z0-9]', '__', node_name) + '.json'


def load_node_summaries(output_dir):
    excluded = {GRAPH_JSON_FILE, NODES_JSON_FILE, DATATYPES_JSON_FILE, NODE_SUMMARIES_JSON_FILE}
    node_summaries = []
    for filename in os.listdir(output_dir):
        if filename.endswith('.json') and filename not in excluded:
            filepath = os.path.join(output_dir, filename)
            with open(filepath, 'r') as f:
                node_summaries.append(json.load(f))
    return node_summaries


def main():
    description = """
    ROS 2 Node Inspector
    The ROS2 Node Inspector (ros2-node-inspector) is a procedural
    command line aggregator for the ROS 2 cli.

    The ROS 2 Node inspector invokes and parses the output from the following
    ROS commands:
        ros2 node list
        ros2 topic info <topic name>
        ros2 node info <node name>
        ros2 interface show <message type>

    All ros2 cli command output is parsed into JSON and placed in
    `ROS_HOME/ros2_node_inspector`.

    USAGE
    To inspect a single node invoke the `ros2-node-inspector` with the node name:
        ros2-node-inspector <node name>

    To inspect all running nodes invoke `ros2-node-inspector` with no arguments:
        ros2-node-inspector
    """

    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('node_name', nargs='?', help='Name of the ROS 2 node to inspect (optional)')
    parser.add_argument('-o', '--output-json-file', help='Optional output JSON file')
    parser.add_argument('-l', '--load-node-summaries', action='store_true', help='Load existing node summaries from output directory')

    args = parser.parse_args()

    output_dir = OUTPUT_DIRECTORY
    os.makedirs(output_dir, exist_ok=True)

    node_summaries = []

    if args.load_node_summaries:
        node_summaries = load_node_summaries(output_dir)
        if not node_summaries:
            print("No node summaries found in the output directory.")
            sys.exit(1)
    elif args.node_name:
        node_name = args.node_name
        output_file = args.output_json_file or os.path.join(output_dir, sanitize_filename(node_name))

        if not os.access(os.path.dirname(output_file), os.W_OK):
            print(f"Error: Cannot write to the specified file: {output_file}")
            sys.exit(1)

        node_summary = ROS2Tools.get_node_summary(node_name)
        write_json_file(output_file, node_summary)
        node_summaries.append(node_summary)
    else:
        nodes = ROS2Tools.get_nodes()
        if not nodes:
            print("The nodes list is empty. Run some ROS nodes and try again.")
            sys.exit(0)
        for node_name in nodes:
            output_file = os.path.join(output_dir, sanitize_filename(node_name))
            if not os.access(os.path.dirname(output_file), os.W_OK):
                print(f"Error: Cannot write to the specified file: {output_file}")
                continue
            node_summary = ROS2Tools.get_node_summary(node_name)
            node_summaries.append(node_summary)
            write_json_file(output_file, node_summary)

    datatypes = []
    for node in node_summaries:
        if not node:
            continue
        for topic in node['topics']:
            if not any(dt["datatype"] == topic['datatype'] for dt in datatypes):
                datatypes.append(copy.deepcopy(topic))

    datatypes = delete_key_recursive(datatypes, "topic")
    datatypes = delete_key_recursive(datatypes, "role")

    graph = ROS2Tools.generate_graph(node_summaries)
    write_json_file(os.path.join(output_dir, GRAPH_JSON_FILE), graph)

    write_json_file(os.path.join(output_dir, NODE_SUMMARIES_JSON_FILE), node_summaries)

    # Write nodes.json with interface fields stripped
    nodes_output = copy.deepcopy(node_summaries)
    delete_key_recursive(nodes_output, "interface")
    delete_key_recursive(nodes_output, "interface_text")
    write_json_file(os.path.join(output_dir, NODES_JSON_FILE), nodes_output)

    write_json_file(os.path.join(output_dir, DATATYPES_JSON_FILE), datatypes)

    graph_json_path = os.path.join(output_dir, GRAPH_JSON_FILE)
    grapher = ROS2NodeGrapher(graph_json_path)
    grapher.create_graph()
    grapher.create_directed_graph()
    grapher.generate_dot_graph()


if __name__ == '__main__':
    main()
