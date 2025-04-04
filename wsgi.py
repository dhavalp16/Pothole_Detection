from app import create_app, socketio
import logging

app = create_app()

if __name__ == '__main__':
    # Configure logging
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    
    print("\n" + "="*50)
    print("🚀 Server starting on http://localhost:5000")
    print("="*50 + "\n")
    
    socketio.run(app, 
                 host='0.0.0.0', 
                 port=5000, 
                 debug=True, 
                 log_output=True,  # Enable log output for debugging
                 use_reloader=False)  # Disable reloader for clearer logs
