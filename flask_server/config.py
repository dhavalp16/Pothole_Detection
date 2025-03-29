import os
from pathlib import Path

basedir = Path(__file__).parent.resolve()
instance_path = basedir / "instance"

# Create instance directory if it doesn't exist
instance_path.mkdir(exist_ok=True)

class Config:
    SQLALCHEMY_DATABASE_URI = f'sqlite:///{instance_path}/potholes.db'
    SQLALCHEMY_TRACK_MODIFICATIONS = False
    SECRET_KEY = 'abcdefg'  # Replace with actual key