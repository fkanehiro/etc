$ sudo apt-get install zsh python-virtualenv guake tmux libclang-dev clang
$ pip install --user powerline-status
$ cd 
$ ln -s <this directory>/.??* .
$ chsh -s /bin/zsh
$ git clone https://github.com/powerline/fonts.git
$ cd fonts
$ ./install.sh
$ cd ..
$ git clone --recursive https://github.com/Andersbakken/rtags.git
$ cd rtags
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

M-x package-list-packages
M-x package-install RET company-irony
M-x package-install RET company-jedi
M-x package-install RET rtags
M-x irony-install-server
M-x jedi:install-server
