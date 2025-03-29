from datetime import datetime
from . import db

class Detection(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    lat = db.Column(db.Float, nullable=False)
    lon = db.Column(db.Float, nullable=False)
    depth = db.Column(db.Float, nullable=False)
    is_pothole = db.Column(db.Boolean, nullable=False)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)

    def to_dict(self):
        return {
            'id': self.id,
            'lat': self.lat,
            'lon': self.lon,
            'depth': self.depth,
            'is_pothole': self.is_pothole,
            'timestamp': self.timestamp.isoformat()
        }
