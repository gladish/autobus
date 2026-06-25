# Autonomous Vehicle
## Goal : Turn a Tamiya King Yellow 6x6 into a fully autonomous vehicle
The King Yellow is a six wheel drive RC kit from Tamiya. I'll skip all the nostalgia about Tamiya RC models, but those who have built any of these kits know the enjoyment of putting together a simple gearbox, hooking up the electronics, and having endless hours of fun trashing it. They're pretty much indesctructable. 

Here's a picture from Tamiya (Stock Photo)

![The original bus](docs/images/stock_king_yellow.png)

We're going to take one of these trucks (bus) and convert it into a fully autonomous vehicle. But before we beging, let's take a look at what we have to work with. A few things to note about this bus
* It's pretty much stock
* We did install a camera from a quadcopter/drone. This is totally seperate from the stock electronics and using it's own lithium battery as a power supply. This was done so we could run it normally (radio controlled) from the back of our house and drive it around the front while watching the video from a fatshark headset.
* There's a gopro mount on top. I honestly forget what we did with that, but presumably we mounted a gopro on it to record stuff.

![Our base model](docs/images/starter_bus.png)

## Plan 
### Phase 1 - Hardware augmentation
First is to outfit the bus with some new hardware. For this I went back and forth between an Arduino UnoQ and a Raspberry Pi. I'm going with rpi for a few reasons. We're going to need a decent MPU for Linux to run things like OpenCV, Video Streaming, IP network connections, and MCU for driving servos. I just wasn't sure the Arduino had the power on the MPU side. If I could get my hands on a Ventuno Q, I'd 100% use that board. Instead I'm going use a raspberry pi5, which I already own, and connect up a "servo driver" board.
#### Parts List
1. https://learn.adafruit.com/16-channel-pwm-servo-driver
1. https://www.raspberrypi.com/products/raspberry-pi-5
1. https://www.raspberrypi.com/products/camera-module-3
### Phase 2 - Network connectivity
### Pahse 3 - Semi-autonomous around the block
