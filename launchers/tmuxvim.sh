#!/bin/bash

#.vimrc
cat << EOF > ~/.vimrc
set number
set nowrap
EOF

#.tmux.conf
cat << EOF > ~/.tmux.conf
#change ctrl-B to ctrl-A
unbind C-b 
set -g prefix C-a 
bind C-a send-prefix

#set mouse
set -g mouse

bind \\\ split-window -h
bind - split-window -v
unbind '"' 
unbind %
EOF
