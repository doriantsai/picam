## High Quality Pi Camerea Notes

- Testing the HQ pi camera, got 'failed to import fd 20' error. Work-around was to use the command 'libcamera-hello --qt-preview'
- HQ pi camera works, preview time default 5 seconds
- 'libcamera-hello --qt-preview -t 0' sets preview time to infinity  
- 'libcamera-jpeg --qt-preview -o test.jpg' captures image (very slow)


# Settings
- `sudo date -s 'YYYY-MM-DD HH:MM:SS'` for setting the time directly, since Pi time is often wrong and internet connection doesn't always manage it
- `sudo update`
- `sudo upgrade`
- sudo rpi-update??
- `sudo service ssh start` enable SSH for remote access
- 
- Enable legacy picamera stack `sudo raspi-confi` into 'interface' and enable camera stack (better support online, picamera2 python stack which uses the more advanced libcamera package looks like it is still under some development for python). Keep tabs on this and consider porting in mid-2023.
- python 3.9, so when calling files, use python3
- raspberry pi OS 11, bullseye
- use system directly since single-purpose pi setup
- wifi driver issues on Model 2, but should be alleviated with Model 4 that has built-in wifi
- 

