from flask import Flask, Response
from picamera2 import Picamera2

import cv2
import time

# python -m flask --app stream run --host=10.10.1.10 --port=10001

app = Flask(__name__)

camera = Picamera2()
camera.configure(
    camera.create_preview_configuration(
        main={"format": "RGB888", "size": (320, 240)}
    )
)
camera.start()


def generate_frames():
    target_fps = 10
    frame_interval = 1.0 / target_fps
    while True:
        start = time.time()
        frame = camera.capture_array()
        ret, buff = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 10])
        frame = buff.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )

        elapsed = time.time() - start
        time.sleep(max(0, (frame_interval - elapsed)))


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
