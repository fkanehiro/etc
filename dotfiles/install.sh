#!/bin/bash

sudo apt-get install zsh python-virtualenv guake tmux libclang-dev clang bear
pip install --user powerline-status

git clone https://github.com/powerline/fonts.git
cd fonts
./install.sh
cd ..

git clone --recursive https://github.com/Andersbakken/rtags.git
cd rtags
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..

THISDIR=$PWD
cd 
ln -s $THISDIR/.??* .
chsh -s /bin/zsh
