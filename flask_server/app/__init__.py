from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_socketio import SocketIO

db = SQLAlchemy()
socketio = SocketIO()

def create_app():
    app = Flask(__name__, template_folder='../templates')  # Explicit path
    # Load configuration
    app.config.from_object('config.Config')
    
    # Initialize extensions
    db.init_app(app)
    socketio.init_app(app)
    
    with app.app_context():
        # Register blueprints/views
        from . import routes
        routes.register_routes(app)
        
        # Create database tables
        db.create_all()
    
    return app