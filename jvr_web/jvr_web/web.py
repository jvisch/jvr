import flask

import rclpy
import rclpy.node

import jvr_helpers.utils


node = None


class Web(rclpy.node.Node):

    def __init__(self):
        node_name = jvr_helpers.utils.node_name(self, hidden_node=True)
        super().__init__(node_name)


def main(args=None):
    global node
    # create ros2 node for interspection
    rclpy.init()
    node = Web()

    testnode = rclpy.node.Node('test_name', namespace='test_namespace')

    # create Flask and configure it
    app = flask.Flask(__name__)
    app.jinja_env.trim_blocks = True
    app.jinja_env.lstrip_blocks = True

    # a simple page that says hello
    @app.route('/hello')
    def hello():
        return 'Hello, World!'

    # from . import auth
    # app.register_blueprint(auth.bp)

    # from . import blog
    # app.register_blueprint(blog.bp)
    # app.add_url_rule('/', endpoint='index')

    from . import nodes
    app.register_blueprint(nodes.bp)


    app.run(debug=True)


if __name__ == '__main__':
    main()
