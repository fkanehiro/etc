$ ./install.sh

M-x package-list-packages
M-x package-install RET company-irony
M-x package-install RET company-jedi
M-x package-install RET rtags
M-x package-install RET powerline
M-x irony-install-server
M-x jedi:install-server

start rtags server
$ rdm --daemon

generate rtags database
for cmake projects (need to enable CMAKE_EXPORT_COMPILE_COMMANDS)
rc -J .
for other projects
bear make
