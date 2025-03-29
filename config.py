import os
from pathlib import Path
from dotenv import load_dotenv

load_dotenv()

basedir = Path(__file__).parent.resolve()
instance_path = basedir / "instance"
instance_path.mkdir(exist_ok=True)

class Config:
    # Use Neon (PostgreSQL) if DATABASE_URL is set; otherwise, fallback to SQLite
    SQLALCHEMY_DATABASE_URI = os.environ.get("DATABASE_URL", f"sqlite:///{instance_path / 'potholes.db'}")
    SQLALCHEMY_TRACK_MODIFICATIONS = False
    SECRET_KEY = os.environ.get("SECRET_KEY", "abcdefg")  # Set your secret key in env vars for production
