#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
touch ~/.vimrc
cat "set number" >> ~/.vimrc
cat "set nowrap" >> ~/.vimrc

touch ~/.tmux.conf
cat "unbind C-b" >> ~/.tmux.conf
cat "set -g prefix C-a" >> ~/.tmux.conf
cat "bind C-a send-prefix" >> ~/.tmux.conf
cat "set -g mouse" >> ~/.tmux.conf
cat "bind \\ split-window -h"  >> ~/.tmux.conf
cat "bind - split-window -v" >> ~/.tmux.conf
cat "unbind \"" >> ~/.tmux.conf
cat "unbind %" >> ~/.tmux.conf

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec echo "This is an empty launch script. Update it to launch your application."


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
