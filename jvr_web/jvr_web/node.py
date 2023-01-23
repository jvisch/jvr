from . import web

import rclpy.action
import flask


bp = flask.blueprints.Blueprint('node', __name__, url_prefix='/node')


@bp.route('/')
@bp.route('/list')
def list():
    n = web.web_node

    def _ns(namespace):
        return None if namespace == '/' else namespace[1:]

    all_nodes = n.get_node_names_and_namespaces()
    all_nodes = ((name, _ns(namespace)) for name, namespace in all_nodes)
    all_nodes = sorted(all_nodes)
    return flask.render_template('node/list.html', nodes=all_nodes)


@bp.route('/info/<node_name>', defaults={'namespace': ''})
@bp.route('/info/<namespace>/<node_name>')
def info(node_name, namespace):
    n = web.web_node
    # namespaces need prefix '/'
    namespace = '/' + namespace
    # 1. Subscribers
    subscribers = n.get_subscriber_names_and_types_by_node(
        node_name, namespace)
    # 2. Publishers
    publishers = n.get_publisher_names_and_types_by_node(node_name, namespace)

    # 3. Service Servers
    service_servers = n.get_service_names_and_types_by_node(
        node_name, namespace)
    # 4. Service Clients
    service_clients = n.get_client_names_and_types_by_node(
        node_name, namespace)

    # 5. Action Servers
    action_servers = rclpy.action.get_action_server_names_and_types_by_node(n, node_name, namespace)
    # 6. Action Clients
    action_clients =  rclpy.action.get_action_client_names_and_types_by_node(n, node_name, namespace)

    full_name = '/' + node_name
    if namespace != '/':
        full_name = namespace + full_name

    return flask.render_template('node/info.html',
                                 node_name=full_name,
                                 subscribers=subscribers,
                                 publishers=publishers,
                                 service_servers=service_servers,
                                 service_clients=service_clients,
                                 action_servers=action_servers,
                                 action_clients=action_clients
                                 )
