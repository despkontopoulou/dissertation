from flask import Flask

from routes.home import home_bp
from routes.pathfinding_routes import pathfinding_bp

app = Flask(__name__)
app.register_blueprint(home_bp)
app.register_blueprint(pathfinding_bp, url_prefix='/pathfinding')

if __name__ == '__main__':
    app.run()
