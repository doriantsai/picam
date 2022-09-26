# High Quality Pi Camerea Notes

- Testing the HQ pi camera, got 'failed to import fd 20' error. Work-around was to use the command 'libcamera-hello --qt-preview'
- HQ pi camera works, preview time default 5 seconds
- 'libcamera-hello --qt-preview -t 0' sets preview time to infinity  
- 'libcamera-jpeg --qt-preview -o test.jpg' captures image (very slow)


# Useful Commands
- `cat /etc/os-release` to make sure correct OS is installed (should be version 10 "buster")
- `raspistill -o name.jpg` to take a photo, check if the camera works, a window should pop up and capture an image
- `gpicview name.jpg` to view the image (like eye of gnome)


# Settings
- Use raspberry pi imager for Raspbian OS Buster (legacy) -> has documented ROS Noetic support, while bullseye does not yet (http://packages.ros.org/ros/ubuntu/dists/)
- `sudo date -s 'YYYY-MM-DD HH:MM:SS'` for setting the time directly, since Pi time is often wrong and internet connection doesn't always manage it
    `sudo update`
    `sudo upgrade`
- sudo rpi-update??
- `sudo service ssh start` enable SSH for remote access, or `sudo raspi-config` and enable `ssh` and `legacy camera stack`

- Install ROS: https://varhowto.com/install-ros-noetic-raspberry-pi-4/
- Enable legacy picamera stack `sudo raspi-config` into 'interface' and enable camera stack (better support online, picamera2 python stack which uses the more advanced libcamera package looks like it is still under some development for python). Keep tabs on this and consider porting in mid-2023.
- python 3.9, so when calling files, use python3
- raspberry pi OS 11 
- use system directly since single-purpose pi setup
 

# scripts to install:
    sudo apt update
    sudo apt upgrade
    sudo apt install build-essential vim-gtk terminator openssh-server git gcc make cmake pkg-config zip unzip g++ curl dkms wget exfat-fuse exfat-utils guvcview net-tools ffmpeg cheese -y

# setup ssh
See https://github.com/doriantsai/linux-setup/blob/main/ssh_setup.md

# git config?
    git config --global user.name "Dorian Tsai"
    git config --global user.email dorian.tsai@gmail.com

# clone relevant repos
    git clone git@github.com:doriantsai/picam.git
    git clone git@github.com:doriantsai/cslics.git
    
# vim
    mkdir -p ~/.vim/autoload ~/.vim/bundle && \
    curl -LSso ~/.vim/autoload/pathogen.vim https://tpo.pe/pathogen.vim
# add pathogen/infect line to .vimrc
    echo -e "execute pathogen#infect()" >> ~/.vimrc
    echo -e "syntax on" >> ~/.vimrc
    echo -e "filetype plugin indent on" >> ~/.vimrc
# install vim-sensible
    cd ~/.vim/bundle && \
    git clone https://github.com/tpope/vim-sensible.git
    echo -e "colorscheme murphy" >> ~/.vimrc

# coding nice-to-haves
    pip3 install mypy
    
# vscode
    sudo apt install software-properties-common apt-transport-https -y
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
    sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
    sudo sh -c 'echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
    sudo apt update
    sudo apt install code

# installing machine vision toolbox (Python)

    sudo apt install libatlas-base-dev
    pip3 install --upgrade matplotlib
    pip3 install --upgrade numpy
    
    git clone git@github.com:petercorke/machinevision-toolbox-python.git
    pip3 install -e .