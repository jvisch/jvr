from . import web

import flask


bp = flask.blueprints.Blueprint('node', __name__, url_prefix='/node')


@bp.route('/')
@bp.route('/list')
def list():
    all_nodes = web.node.get_node_names_and_namespaces()

    # inspired on: https://github.com/ros2/ros2cli/blob/13870ad51cb8d52dd7171407793ae0198dcc6ac4/ros2node/ros2node/api/__init__.py#L65
    def _node_name(name, namespace):
        ns = namespace
        if not namespace.endswith('/'):
            ns += '/'
        return ns + name
    all_nodes = (_node_name(n, ns) for n, ns in all_nodes)
    all_nodes = sorted(all_nodes)
    return flask.render_template('node/list.html', nodes=all_nodes)

# <path:pars>
@bp.route('/info/<node_name>', defaults={'namespace': ''})
@bp.route('/info/<namespace>/<node_name>')
def info(node_name, namespace):
    n = web.node
    # namespaces need prefix '/'
    namespace = '/' + namespace
    # 1. Subscribers: get_subscriber_names_and_types_by_node
    subscribers = n.get_subscriber_names_and_types_by_node(node_name, namespace)
    return subscribers
    # 2. Publishers:  get_publisher_names_and_types_by_node

    # 3. Service Servers: get_service_names_and_types_by_node
    # 4. Service Clients: get_client_names_and_types_by_node
    
    # 5. Action Servers: get_action_server_names_and_types_by_node
    # 6. Action Clients: get_action_client_names_and_types_by_node