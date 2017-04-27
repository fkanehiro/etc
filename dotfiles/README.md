$ sudo apt-get install zsh python-virtualenv guake tmux libclang-dev
$ pip install powerline-status
$ cd 
$ ln -s <this directory>/.??* .

M-x package-list-packages
M-x package-install RET company-irony
M-x package-install RET company-jedi
M-x irony-install-server
M-x jedi:install-server
