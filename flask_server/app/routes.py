from flask import request, jsonify, render_template
from . import db, socketio
from .models import Detection

def register_routes(app):
    @app.route('/')
    def index():
        return render_template('map.html')

    @app.route('/api/detections', methods=['POST'])
    def add_detection():
        data = request.get_json()
        try:
            detection = Detection(
                lat=data['lat'],
                lon=data['lon'],
                depth=data['depth'],
                is_pothole=data['is_pothole']
            )
            db.session.add(detection)
            db.session.commit()
            socketio.emit('new_detection', detection.to_dict())
            return jsonify(detection.to_dict()), 201
        except KeyError as e:
            return jsonify({'error': f'Missing field: {str(e)}'}), 400