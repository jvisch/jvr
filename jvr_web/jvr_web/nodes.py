from . import web

import flask.blueprints

bp = flask.blueprints.Blueprint('nodes', __name__, url_prefix= '/nodes')

@bp.route('list')
def list():
    return web.node.get_node_names_and_namespaces()