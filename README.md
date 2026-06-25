# Autonomous Vehicle
## Goal : Turn a Tamiya King Yellow 6x6 into a fully autonomous vehicle
The King Yellow is a six wheel drive RC kit from Tamiya. I'll skip all the nostalgia about Tamiya RC models, but if I've built any of these kits, I know the enjoyment of putting together a simple vehicle and then seeing it run. This project is about taking that experience in a very different direction.

Here's a picture from Tamiya (Stock Photo)

![The original bus](docs/images/stock_king_yellow.png)

I'm going to take one of these trucks (bus) and convert it into a fully autonomous vehicle. But before I begin, let's take a look at what I have to work with. A few things to note about this bus:
* It's pretty much stock
* I did install a camera from a quadcopter/drone. This is totally separate from the stock electronics and uses its own lithium battery as a power supply. I did this so I could run it normally without interfering with the stock setup.
* There's a GoPro mount on top. I honestly forget what I did with that, but presumably I mounted a GoPro on it to record stuff.

![Our base model](docs/images/starter_bus.png)

## Plan 
### Phase 1 - Hardware augmentation
First is to outfit the bus with some new hardware. For this I went back and forth between an Arduino UnoQ and a Raspberry Pi. I'm going with an RPi for a few reasons. I'm going to need a decent MPU for vision, networking, and control, and the Pi seems like the better fit.

#### Functional Requirements
* The bus can boot Linux and I can control some servos with software on Linux.
* I can attach Ethernet for networking.
* The servo driver is connected to the RPi.
* The servos do not have to be bus servos. The bus has electronic speed control.

#### Parts List
1. https://learn.adafruit.com/16-channel-pwm-servo-driver
1. https://www.raspberrypi.com/products/raspberry-pi-5
1. https://www.raspberrypi.com/products/camera-module-3
1. 5200 mAh Anker USB Power Bank.
   
I'm not sure how long that power bank will last, but I have a USB power meter that I'll use later on to figure out how long the Pi will run before it shuts off due to lack of power. That's certainly something I'll need to know. 

Here's the rpi5 booted from the Anker
![BatteryPowered](docs/images/battery_powered_pi.png)


### Phase 3 - Network connectivity
I want connectivity between the bus and my home LAN. This will allow me to monitor the bus, take over manually, and stream video back.
#### Functional Requirements
* My desktop PC (Windows 11) is connected to the bus over WAN (internet).
* My Linux LAN PC can connect to the bus over WAN (SSH, MQTT).
* I can stream video from the Pi camera to my Windows 11 PC over WAN.
#### iPhone Tether
The plan is to tether my iPhone to the RPi over USB and enable hotspot on the phone. It's slightly awkward that I have to enable hotspot to get connectivity over USB, but that's apparently how the iPhone handles it.
#### VPN
Once the bus has internet connectivity, I'll connect everything on one network via a VPN. I'm going to use Tailscale for this. It was the first zero cost VPN that I found and it's super easy to set up.

#### Network Topology

* My RPi5 (bus) gets its internet via USB tethering to my iPhone, which uses cellular data.
* Once online, my RPi5 joins my Tailscale mesh as a node.
* My Windows 11 desktop and the Linux relay server (on my home network) are also Tailscale nodes in that same mesh.
* Tailscale builds direct encrypted tunnels between peers when possible, and the relay server — sitting on my home LAN with stable connectivity — can act as a fallback relay/exit node for the bus if needed.

![Network Topology](docs/images/bus_tailscale_network_diagram.png)

#### Intended Use

This network layout is meant to support a few different tasks:

* **Remote shell access** to the Raspberry Pi over SSH
* **Telemetry and messaging** between the bus and other systems, likely over MQTT
* **Manual intervention or monitoring** from my Windows desktop
* **Video streaming** from the Pi camera back to my workstation
* **Relay or bridge services** hosted from the Linux machine on my home network

Using Tailscale means each machine can communicate over a private VPN without me needing to expose services directly to the public internet.

### Pahse 4 - OpenCV Vision Integration
#### Functional Requirements
* The bus can navigate itself around the block without manual steering
