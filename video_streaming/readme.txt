README

** NOTE:
- On PC/Laptop and Raspi:   . Linux environment
                            . python3 installed
                            . OpenCV 3.4 installed
                            . netcat packed installed
                            . ffmpeg packed installed
                            . sudo modprobe bcm2835-v4l2 (enable Camera)
** Using guide:
* Connect PC/Laptop and Raspi to a Wifi network
* On PC/Laptop side: 
- Create a FIFO streaming by using command: ./mkfifo.sh
- Get video streaming by running python script: python3 getVideoStream.py
* On Raspi side:
- Change IP in the ncstream0.sh to PC/Laptop IP
- Start streaming by running ncstream0 script: ./ncstream0.sh