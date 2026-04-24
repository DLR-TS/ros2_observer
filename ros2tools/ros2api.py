import threading
import time
import logging
import sys
import os
from flask import Blueprint, jsonify, request

try:
    from ros2tools.util import *
except ImportError:
    sys.path.append(os.path.join(os.path.dirname(__file__), "."))
    from .util import *
from ros2tools import ROS2Tools

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

__all__ = [
    'ROS2API',
    'ROS2Cache', 
    'ROS2Worker',
    'get_ros2tools_blueprint',
    'stop_ros2_worker',
    'ros2_api',
    'ros2tools_blueprint'
]

class ROS2Cache:
    def __init__(self):
        self.nodes = {}
        self.topics = {}
        self.topic_datatypes = {}
        self.datatypes = {}
        self.graph = None
        self.last_updated = {}
        self.lock = threading.RLock()
    
    def update_nodes(self, nodes_data):
        with self.lock:
            self.nodes = nodes_data
            self.last_updated['nodes'] = time.time()
    
    def update_topics(self, topics_data):
        with self.lock:
            self.topics = topics_data
            self.last_updated['topics'] = time.time()

    def update_topic_datatypes(self, datatypes_map):
        with self.lock:
            self.topic_datatypes = datatypes_map
            self.last_updated['topic_datatypes'] = time.time()

    def update_datatypes(self, datatypes_data):
        with self.lock:
            self.datatypes = datatypes_data
            self.last_updated['datatypes'] = time.time()
    
    def update_graph(self, graph_data):
        with self.lock:
            self.graph = graph_data
            self.last_updated['graph'] = time.time()
    
    def get_data(self, data_type):
        with self.lock:
            return getattr(self, data_type, {}).copy()

class ROS2Worker:
    def __init__(self, cache, update_interval=10):
        self.cache = cache
        self.update_interval = update_interval
        self.running = False
        self.thread = None
    
    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.thread.start()
            logger.info("ROS2 worker thread started")
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
    
    def _worker_loop(self):
        while self.running:
            try:
                self._update_cache()
                time.sleep(self.update_interval)
            except Exception as e:
                logger.error(f"Error in worker loop: {e}")
                time.sleep(self.update_interval)
    
    def _update_cache(self):
        try:
            # Fast live topic list with datatypes — independent of node inspection
            try:
                topic_datatypes = ROS2Tools.get_topics()
                self.cache.update_topic_datatypes(topic_datatypes)
            except Exception as e:
                logger.error(f"Error fetching topic list: {e}")

            node_names = ROS2Tools.get_nodes()
            if not node_names:
                return
            
            nodes_data = {}
            topics_data = {}
            datatypes_data = {}
            
            for node_name in node_names:
                if not node_name.strip():
                    continue
                
                try:
                    node_summary = ROS2Tools.get_node_summary(node_name)
                    if node_summary:
                        nodes_data[node_name] = node_summary
                        
                        for topic_info in node_summary.get('topics', []):
                            topic_name = topic_info['topic']
                            datatype = topic_info['datatype']
                            
                            if topic_name not in topics_data:
                                topics_data[topic_name] = {
                                    'name': topic_name,
                                    'datatype': datatype,
                                    'publishers': [],
                                    'subscribers': []
                                }
                            elif datatype and not topics_data[topic_name].get('datatype'):
                                topics_data[topic_name]['datatype'] = datatype
                            
                            role = topic_info['role']
                            if role == 'publisher':
                                topics_data[topic_name]['publishers'].append(node_name)
                            elif role == 'subscriber':
                                topics_data[topic_name]['subscribers'].append(node_name)
                            
                            if datatype not in datatypes_data:
                                datatypes_data[datatype] = {
                                    'name': datatype,
                                    'interface_text': topic_info.get('interface_text', ''),
                                    'interface': topic_info.get('interface', []),
                                    'topics': []
                                }
                            
                            if topic_name not in datatypes_data[datatype]['topics']:
                                datatypes_data[datatype]['topics'].append(topic_name)
                
                except Exception as e:
                    logger.error(f"Error processing node {node_name}: {e}")
                    continue
            
            graph_data = None
            if nodes_data:
                try:
                    node_summaries = list(nodes_data.values())
                    graph_data = ROS2Tools.generate_graph(node_summaries)
                except Exception as e:
                    logger.error(f"Error generating graph: {e}")
            
            self.cache.update_nodes(nodes_data)
            self.cache.update_topics(topics_data)
            self.cache.update_datatypes(datatypes_data)
            if graph_data:
                self.cache.update_graph(graph_data)
            
            logger.info(f"Cache updated: {len(nodes_data)} nodes, {len(topics_data)} topics, {len(datatypes_data)} datatypes")
            
        except Exception as e:
            logger.error(f"Error updating cache: {e}")

class ROS2API:
    def __init__(self, update_interval=10):
        self.cache = ROS2Cache()
        self.worker = ROS2Worker(self.cache, update_interval)
        self.blueprint = self._create_blueprint()
        self.worker.start()
    
    def _create_blueprint(self):
        bp = Blueprint('ros2_blueprint', __name__, url_prefix='/api/ros2')
        
        @bp.route('/nodes', methods=['GET'])
        def get_nodes():
            nodes = self.cache.get_data('nodes')
            return jsonify({
                'nodes': nodes,
                'count': len(nodes),
                'last_updated': self.cache.last_updated.get('nodes')
            })

        @bp.route('/nodes/running', methods=['GET'])
        def get_running_nodes():
            nodes = ROS2Tools.get_nodes()
            return jsonify({
                'running_nodes': nodes,
                'count': len(nodes)
            })
 

        @bp.route('/nodes/<node_name>', methods=['GET'])
        def get_node(node_name):
            nodes = self.cache.get_data('nodes')
            
            if node_name.startswith('/'):
                target_node = node_name
            else:
                target_node = None
                for name in nodes.keys():
                    if name.endswith(node_name) or name == f"/{node_name}":
                        target_node = name
                        break
            
            if target_node and target_node in nodes:
                return jsonify(nodes[target_node])
            else:
                return jsonify({'error': f'Node {node_name} not found'}), 404
        
        @bp.route('/topics', methods=['GET'])
        def get_topics():
            topics = self.cache.get_data('topics')
            topic_datatypes = self.cache.get_data('topic_datatypes')
            return jsonify({
                'topics': topics,
                'topic_datatypes': topic_datatypes,
                'count': len(topics),
                'last_updated': self.cache.last_updated.get('topics')
            })

        @bp.route('/topics/list', methods=['GET'])
        def get_topics_live():
            """Live ros2 topic list -t — no cache."""
            try:
                topic_datatypes = ROS2Tools.get_topics()
                return jsonify({
                    'topic_datatypes': topic_datatypes,
                    'topics': sorted(topic_datatypes.keys()),
                    'count': len(topic_datatypes)
                })
            except Exception as e:
                return jsonify({'error': str(e)}), 500

        @bp.route('/topics/<path:topic_name>', methods=['GET'])
        def get_topic(topic_name):
            if not topic_name.startswith('/'):
                topic_name = f"/{topic_name}"
            topics = self.cache.get_data('topics')
            if topic_name in topics:
                return jsonify(topics[topic_name])
            return jsonify({'error': f'Topic {topic_name} not found'}), 404

        @bp.route('/topics/<path:topic_name>/datatype', methods=['GET'])
        def get_topic_datatype(topic_name):
            if not topic_name.startswith('/'):
                topic_name = f"/{topic_name}"
            # Check cache first, fall back to live query
            topic_datatypes = self.cache.get_data('topic_datatypes')
            datatype = topic_datatypes.get(topic_name)
            if datatype is None:
                datatype = ROS2Tools.get_topic_datatype(topic_name)
            return jsonify({'topic': topic_name, 'datatype': datatype})

        @bp.route('/topics/<path:topic_name>/hz', methods=['POST'])
        def get_topic_hz(topic_name):
            if not topic_name.startswith('/'):
                topic_name = f"/{topic_name}"
            try:
                output = ROS2Tools.topic_hz(topic_name)
                return jsonify({'topic': topic_name, 'output': output})
            except Exception as e:
                return jsonify({'error': str(e)}), 500

        @bp.route('/interface/<path:datatype>/proto', methods=['GET'])
        def get_interface_proto(datatype):
            try:
                proto = ROS2Tools.get_interface_proto(datatype)
                return jsonify({'datatype': datatype, 'proto': proto})
            except Exception as e:
                return jsonify({'error': str(e)}), 500
        
        @bp.route('/datatypes', methods=['GET'])
        def get_datatypes():
            datatypes = self.cache.get_data('datatypes')
            return jsonify({
                'datatypes': datatypes,
                'count': len(datatypes),
                'last_updated': self.cache.last_updated.get('datatypes')
            })
        
        @bp.route('/datatypes/<path:datatype_name>', methods=['GET'])
        def get_datatype(datatype_name):
            datatypes = self.cache.get_data('datatypes')
            if datatype_name in datatypes:
                return jsonify(datatypes[datatype_name])
            else:
                return jsonify({'error': f'Datatype {datatype_name} not found'}), 404
        
        @bp.route('/graph', methods=['GET'])
        def get_graph():
            graph = self.cache.get_data('graph')
            return jsonify({
                'graph': graph,
                'last_updated': self.cache.last_updated.get('graph')
            })
        
        @bp.route('/status', methods=['GET'])
        def get_status():
            with self.cache.lock:
                return jsonify({
                    'worker_running': self.worker.running,
                    'last_updated': self.cache.last_updated,
                    'cache_stats': {
                        'nodes': len(self.cache.nodes),
                        'topics': len(self.cache.topics),
                        'topic_datatypes': len(self.cache.topic_datatypes),
                        'datatypes': len(self.cache.datatypes)
                    }
                })
        
        @bp.route('/refresh', methods=['POST'])
        def force_refresh():
            try:
                self.worker._update_cache()
                return jsonify({'message': 'Cache refreshed successfully'})
            except Exception as e:
                return jsonify({'error': str(e)}), 500
        
        return bp
    
    def get_blueprint(self):
        return self.blueprint
    
    def stop_worker(self):
        self.worker.stop()

ros2_api = ROS2API()
ros2tools_blueprint = ros2_api.get_blueprint()

def get_ros2tools_blueprint():
    return ros2_api.get_blueprint()

def stop_ros2_worker():
    ros2_api.stop_worker()
