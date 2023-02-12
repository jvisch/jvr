import flask


bp = flask.blueprints.Blueprint('index', __name__)

@bp.route('/')
def index():
    hyperlinks = [
        ('Nodes', flask.url_for('node.index')),
        ('Topics', flask.url_for('topic.index'))
        ]
    return flask.render_template('index/index.jinja', links=hyperlinks)