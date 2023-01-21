from . import web

import flask


bp = flask.blueprints.Blueprint('nodes', __name__, url_prefix='/nodes')


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
    return flask.render_template('nodes/list.html', nodes=all_nodes)
