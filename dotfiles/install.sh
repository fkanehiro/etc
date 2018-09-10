#!/bin/bash -e

OSNAME="$(uname)"

if [ $OSNAME = "Darwin" ]; then
    /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
    brew install caskbrew emacs git tmux
else
    sudo apt-get install python-virtualenv guake tmux libclang-dev clang bear cmake emacs python-pip xsel
fi

pip install --user powerline-status

git clone https://github.com/powerline/fonts.git
cd fonts
./install.sh
cd ..

if [ $OSNAME = "Darwin" ]; then
    sudo cp omniORB.cfg /etc/
else
    git clone --recursive https://github.com/Andersbakken/rtags.git
    cd rtags
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    cd ../..
fi

THISDIR=$PWD
cd 

if [ $OSNAME = "Darwin" ]; then
    ln -s $THISDIR/.emacs.el .
    ln -s $THISDIR/.bash_profile .
    ln -s $THISDIR/.tmux.conf.macos .tmux.conf
else
    mv .bashrc .bashrc.bak
    ln -s $THISDIR/.bashrc .
    ln -s $THISDIR/.emacs .
    ln -s $THISDIR/.tmux.conf .
    ln -s $THISDIR/.tmux.d .
fi
