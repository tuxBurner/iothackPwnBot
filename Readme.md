## Motor Controller Doku

* https://shop.pimoroni.com/products/motor-2040?variant=39884997853267
* https://github.com/pimoroni/pimoroni-pico/tree/main/micropython/examples/motor2040
* https://github.com/pimoroni/pimoroni-pico

Starting MicroPython: https://learn.pimoroni.com/article/getting-started-with-pico

### Setup
1. sudo raspi-config (unter Interfaces, remote GPIO einschalten und pi neustarten)
2. sudo pigpiod (deamon starten)
3. Berechtigung auf USB Port für Motor sicherstellen (chmod 777...)
4. eventuell websocket in html anpassen
5. sudo apt-get install python3-opencv
6. sudo sh startup.sh
