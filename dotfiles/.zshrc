export PATH=$HOME/openrtp/bin:$HOME/.local/bin:$PATH
export LD_LIBRARY_PATH=$HOME/openrtp/lib:$HOME/openrtp/lib64:/usr/local/lib/roboptim-core
export PYTHONPATH=$HOME/openrtp/lib/python2.7/dist-packages/hrpsys
export PKG_CONFIG_PATH=$HOME/openrtp/lib/pkgconfig

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

powerline-daemon -q
. /home/kanehiro/.local/lib/python2.7/site-packages/powerline/bindings/zsh/powerline.zsh

[[ -z "$TMUX" && ! -z "$PS1" ]] && tmux
