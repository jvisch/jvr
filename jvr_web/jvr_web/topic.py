from . import web
from . import helpers

import flask


bp = flask.blueprints.Blueprint('topic', __name__, url_prefix='/topic')


@bp.route('/')
def index():
    return flask.redirect(flask.url_for('topic.list'))


@bp.route('/list')
def list():
    n = web.web_node

    topics = n.get_topic_names_and_types()
    return flask.render_template('topic/list.jinja', topics=topics)


@bp.route('/info/<path:topic_name>')
def info(topic_name):
    n = web.web_node

    topic = '/' + topic_name
    publishers = n.get_publishers_info_by_topic(topic)
    subscriptions = n.get_subscriptions_info_by_topic(topic)
    return flask.render_template('topic/info.jinja',
                                 topic_name=topic,
                                 publishers=publishers,
                                 subscriptions=subscriptions
                                 )
