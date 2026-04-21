import subprocess
import json
import yaml
import sys
import re
import os
import time

try:
    from ros2tools.util import *
    from ros2tools.util import run_command

except ImportError:
    sys.path.append(os.path.join(os.path.dirname(__file__), "."))
    from util import *
    from util import run_command
    from trace_converter import *


EMPTY_MESSAGE_COMMAND_TEMPLATE = 'ros2 topic pub -1 {topic} {data_type} "{{}}" >/dev/null 2>&1 && sleep 1s && ros2 topic echo {topic} --once | grep -v "WARNING"'
EMPTY_MESSAGE_COMMAND_TEMPLATE_RAW = 'ros2 topic pub -1 {topic} {data_type} "{{}}"'

_RETRY_ATTEMPTS = 3
_RETRY_DELAY = 1.0


def _run_with_retry(cmd, attempts=_RETRY_ATTEMPTS, delay=_RETRY_DELAY):
    for i in range(attempts):
        result = run_command(cmd)
        if result and result[0]:
            return result
        if i < attempts - 1:
            time.sleep(delay)
    return result


class ROS2Tools:

    PRIMITIVE_TYPES = [
        "bool",
        "byte",
        "char",
        "float32",
        "float64",
        "int8",
        "uint8",
        "int16",
        "uint16",
        "int32",
        "uint32",
        "int64",
        "uint64",
        "string",
        "wstring"
    ]

    @staticmethod
    def filter_edges(graph):
        node_types = {node['id']: node['type'] for node in graph['nodes']}
        return [
            edge for edge in graph['edges']
            if node_types.get(edge['source']) != 'topic' and node_types.get(edge['target']) != 'topic'
        ]

    @staticmethod
    def find_edge(edges, node_a, node_b, topic):
        edge_a = next((edge for edge in edges if edge['source'] == node_a['node'] and edge['target'] == topic['topic']), None)
        edge_b = next((edge for edge in edges if edge['source'] == topic and edge['target'] == node_b['node']), None)
        if edge_a and edge_b:
            return {"source": node_a['node'], "target": node_b['node'], 'topic': topic}
        return None

    @staticmethod
    def generate_graph(nodes):
        graph = {'nodes': [], 'edges': []}
        publisher_edges = []
        subscriber_edges = []
        consolidated_edges = []

        for node in nodes:
            if not node:
                continue
            graph['nodes'].append({'id': node['node'], 'type': 'node'})
            for topic in node['topics']:
                if topic['role'] == "subscriber":
                    subscriber_edges.append({
                        'source': topic['topic'], 'target': node['node'],
                        'node': node['node'], 'topic': topic['topic'], 'type': 'subscriber'
                    })
                elif topic['role'] == "publisher":
                    publisher_edges.append({
                        'source': node['node'], 'target': topic['topic'],
                        'node': node['node'], 'topic': topic['topic'], 'type': 'publisher'
                    })

        for pub in publisher_edges:
            for sub in subscriber_edges:
                if pub['topic'] == sub['topic'] and pub['source'] != sub['target']:
                    consolidated_edges.append({
                        'source': pub['source'],
                        'target': sub['target'],
                        'topic': pub['topic']
                    })

        graph['edges'] = consolidated_edges
        return graph

    @staticmethod
    def parse_typedef_text(typedef_text, interface_text):
        if not typedef_text or not interface_text:
            return {}

        typedef_pattern = re.compile(
            r'^(?P<datatype>[a-zA-Z0-9_/]+)'
            r'(?P<constraint><=?\d+)?'
            r'(\[(?P<array_constraint>[^\]]*)\])?'
            r'\s+(?P<label>[a-zA-Z0-9_]+)'
            r'(?:\s*=\s*(?P<value>[^\s]+))?'
            r'(?:\s+(?P<value_no_equals>[^\s]+))?'
            r'$'
        )

        match = typedef_pattern.match(typedef_text)
        if not match:
            return None

        datatype = match.group('datatype')
        label = match.group('label')
        array_constraint = match.group('array_constraint') or ""
        value = match.group('value') or match.group('value_no_equals') or ""
        constraint = match.group('constraint') or ""

        if value:
            typedef_type = "constant"
        elif "[]" in typedef_text or array_constraint:
            typedef_type = "array" if datatype in ROS2Tools.PRIMITIVE_TYPES else "object_array"
        elif datatype in ROS2Tools.PRIMITIVE_TYPES:
            typedef_type = "primitive"
        else:
            typedef_type = "object"

        typedef_dict = {
            "type": typedef_type,
            "datatype": datatype,
            "label": label,
            "array_constraint": array_constraint,
            "constraint": constraint,
            "value": value,
            "typedef_text": typedef_text
        }

        if typedef_dict['type'] in ["object", "object_array"]:
            typedef_dict['fields'] = []
            typedef_dict['object_interface_text'] = ROS2Tools.get_object_interface_text(interface_text, typedef_text)
            if typedef_dict['object_interface_text']:
                typedef_dict["fields"] = ROS2Tools.get_fields(typedef_dict['object_interface_text'])

        return typedef_dict

    @staticmethod
    def get_fields(interface_text):
        lines = interface_text.splitlines()
        return [ROS2Tools.parse_typedef_text(line, interface_text) for line in lines if line.strip() and not line.startswith((' ', '\t'))]

    @staticmethod
    def remove_parent_object_nesting(text):
        lines = text.splitlines()
        if not lines:
            return text

        first_line = next((line for line in lines if line.strip()), "")
        leading_whitespace = len(first_line) - len(first_line.lstrip())

        if first_line.startswith(" " * leading_whitespace):
            whitespace_char = ' '
        elif first_line.startswith("\t" * leading_whitespace):
            whitespace_char = '\t'
        else:
            return text

        for line in lines:
            if line.strip() and not line.startswith(whitespace_char * leading_whitespace):
                return text

        adjusted_lines = []
        for line in lines:
            if line.startswith(whitespace_char * leading_whitespace):
                adjusted_lines.append(line[leading_whitespace:])
            else:
                adjusted_lines.append(line)

        return "\n".join(adjusted_lines)

    @staticmethod
    def remove_one_level_of_nesting(text):
        lines = text.splitlines()
        if not lines:
            return text

        first_line = lines[0]
        leading_whitespace = len(first_line) - len(first_line.lstrip())
        if leading_whitespace == 0:
            return text

        if first_line.startswith(" " * leading_whitespace):
            whitespace_char = ' '
        elif first_line.startswith("\t" * (leading_whitespace // len("\t"))):
            whitespace_char = '\t'
        else:
            return text

        adjusted_lines = []
        for line in lines:
            if line.startswith(whitespace_char * leading_whitespace):
                adjusted_lines.append(line[leading_whitespace:])
            else:
                adjusted_lines.append(line)

        return "\n".join(adjusted_lines)

    @staticmethod
    def get_object_interface_text(interface_text, field_text):
        lines = interface_text.splitlines()
        object_lines = []
        counter = -1
        for line in lines:
            if line.strip() == field_text.lstrip():
                counter += 1
                continue
            if counter >= 0:
                if line == line.lstrip():
                    break
                else:
                    object_lines.append(line)

        return ROS2Tools.remove_parent_object_nesting("\n".join(object_lines))

    @staticmethod
    def trim_whitespace_around_equals(input_string):
        parts = input_string.split('=', 1)
        if len(parts) == 1:
            return input_string.strip()
        return f"{parts[0].strip()}={parts[1].strip()}"

    @staticmethod
    def parse_interface_text(interface_text):
        return ROS2Tools.get_fields(interface_text)

    @staticmethod
    def trim_comments(text):
        lines = text.splitlines()
        trimmed_lines = []
        for line in lines:
            stripped_line = line.lstrip()
            if stripped_line.startswith("#"):
                continue
            if "#" in line:
                line = line.split("#", 1)[0].rstrip()
            if line.strip():
                trimmed_lines.append(line)
        return "\n".join(trimmed_lines)

    @staticmethod
    def get_interface_text(message_type):
        output = ROS2Tools.trim_comments(_run_with_retry(f"ros2 interface show {message_type}")[0])
        return output

    @staticmethod
    def get_topic_info(topic_name):
        info = _run_with_retry(f'ros2 topic info {topic_name} | sed "/  Service Servers:/q"')[0]
        if info:
            for line in info.split("\n"):
                if line.startswith("Type:"):
                    return line.split()[1]
        return None

    @staticmethod
    def get_node_info(node_name):
        # node_name from `ros2 node list` is always fully qualified; try as-is first
        cmd_template = "ros2 node info --no-daemon {name} | sed '/Service Servers:/q' | sed '/Service Servers:/d'"
        node_info = _run_with_retry(cmd_template.format(name=node_name))[0]
        if not node_info and not node_name.startswith('/'):
            node_info = _run_with_retry(cmd_template.format(name=f'/{node_name}'))[0]
        if not node_info:
            return None
        print(node_info)
        return node_info

    @staticmethod
    def get_nodes():
        node_list = _run_with_retry("ros2 node list --all --no-daemon | grep -v ros2cli | sort | uniq")[0]
        if not node_list:
            return []
        return [n for n in node_list.split("\n") if n.strip()]

    @staticmethod
    def get_interface_info(message_type):
        return ROS2Tools.trim_comments(_run_with_retry(f"ros2 interface show {message_type}")[0])

    @staticmethod
    def get_node(node_name):
        nodes = ROS2Tools.get_nodes()
        for node in nodes:
            if node.endswith(node_name):
                return node
        available_nodes = "\n".join(nodes)
        raise Exception(
            f"ERROR: No node with the name: '{node_name}' not found. Did you run the scenario? \n"
            f"Available nodes:\n{available_nodes}\nRun a scenario and try again."
        )

    @staticmethod
    def get_node_summary(node_name):
        node_summary = {"node": node_name}
        node_info = ROS2Tools.get_node_info(node_name)
        if not node_info:
            print(f"Failed to get information for node '{node_name}'")
            return None

        topics = []
        is_publisher_section = False
        is_subscriber_section = False

        for line in node_info.strip().split('\n'):
            if 'Publishers:' in line:
                is_publisher_section = True
                is_subscriber_section = False
                continue
            elif 'Subscribers:' in line:
                is_publisher_section = False
                is_subscriber_section = True
                continue

            if not line.strip():
                continue

            parts = line.split(':', 1)
            if len(parts) < 2:
                continue
            topic = parts[0].strip()
            datatype = parts[1].strip()
            if not topic or not datatype:
                continue

            if is_publisher_section or is_subscriber_section:
                role = 'publisher' if is_publisher_section else 'subscriber'
                interface_text = ROS2Tools.get_interface_info(datatype)
                interface = ROS2Tools.parse_interface_text(interface_text)
                topics.append({
                    'topic': topic,
                    'role': role,
                    'datatype': datatype,
                    'interface_text': interface_text,
                    'interface': interface
                })

        node_summary['topics'] = topics
        return node_summary
