from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": (1280, 720)}
)

picam2.configure(config)

encoder = H264Encoder(bitrate=1_000_000)

output = FfmpegOutput(
    "udp://100.86.222.73:5000?pkt_size=1316"
)

picam2.start_recording(encoder, output)
picam2.start()
