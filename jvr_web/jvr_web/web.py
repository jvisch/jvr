import flask
import argparse

import rclpy
import rclpy.node

import jvr_helpers.utils


web_node = None


class Web(rclpy.node.Node):

    def __init__(self):
        node_name = jvr_helpers.utils.node_name(self, hidden_node=True)
        super().__init__(node_name)


def main(args=None):
    global web_node
    # commandline args
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--flask-debug', help='run flask in debug mode',
                        dest='flask_debug', required=False, default=False, action='store_true')
    args_values = parser.parse_args()

    # create ros2 node for interspection
    rclpy.init()
    web_node = Web()

    # testnode = rclpy.node.Node('test_name', namespace='test_namespace')

    # create Flask and configure it
    app = flask.Flask(__name__)
    app.jinja_env.trim_blocks = True
    app.jinja_env.lstrip_blocks = True

    # Import flask functions
    from . import node
    app.register_blueprint(node.bp)
    from . import topic
    app.register_blueprint(topic.bp)
    from . import index
    app.register_blueprint(index.bp)

    # Hello page (for debug)
    if args_values.flask_debug:
        # a simple page that says hello
        @app.route('/hello')
        def hello():
            return 'Hello, World!'

    # Launch Flask
    app.run(
        debug=args_values.flask_debug,
        host="0.0.0.0"
        )


if __name__ == '__main__':
    main()
