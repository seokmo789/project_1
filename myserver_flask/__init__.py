
import os
import sys
from flask import Flask
sys.path.append(os.path.dirname(os.path.abspath(__file__)))




def create_app():
    app=Flask(__name__)

    from views import camera_new
    app.register_blueprint(camera_new.bp)
    return app
    
if __name__ == "__main__":
    app=create_app()
    app.run(host='0.0.0.0', debug=True, port=5000)
'''
    from .views import main_views
    app.register_blueprint(main_views.bp)
'''    



