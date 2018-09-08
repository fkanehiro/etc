export PATH=$HOME/Library/Python/2.7/bin:$PATH
export PATH=/usr/local/opt/gettext/bin:$PATH
export PATH=/usr/local/opt/qt/bin:$PATH

alias ls='ls -FG'
alias grep='grep --color'
alias ldd='otool -L'

. $HOME/src/drcutil/setup.bash 

source /usr/local/etc/bash_completion.d/git-prompt.sh
source /usr/local/etc/bash_completion.d/git-completion.bash

powerline-daemon -q
. /Users/kanehiro/Library/Python/2.7/lib/python/site-packages/powerline/bindings/bash/powerline.sh 

[[ -z "$TMUX" && ! -z "$PS1" ]] && tmux
