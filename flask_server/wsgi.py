from app import create_app, socketio
import logging

app = create_app()

if __name__ == '__main__':
    # Configure logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    
    print("\n" + "="*50)
    print(f"🚀 Server starting on http://localhost:5000")
    print("="*50 + "\n")
    
    socketio.run(app)
