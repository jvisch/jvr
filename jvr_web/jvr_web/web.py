import flask

import rclpy
import rclpy.node

import jvr_helpers.utils


node = None


class Web(rclpy.node.Node):

    def __init__(self):
        print('gemaakt')
        node_name = jvr_helpers.utils.node_name(self, hidden_node=True)
        super().__init__(node_name)


def main(args=None):
    global node
    # create ros2 node for interspection
    rclpy.init()
    node = Web()

    app = flask.Flask(__name__)

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


    app.run()


if __name__ == '__main__':
    main()
