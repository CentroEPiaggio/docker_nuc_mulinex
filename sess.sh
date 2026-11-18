#!/bin/bash
# setup-dev.sh

SESSION="Race-dev"

# Create session
tmux new-session -d -s $SESSION

tmux set-option -g mouse on

tmux split-window -h -t $SESSION:0
tmux split-window -v -t $SESSION

tmux select-pane -t $SESSION:0.0
tmux split-window -v -t $SESSION

tmux select-layout -t $SESSION tiled
tmux set-option  status-style "bg=red"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
README_FILE="$SCRIPT_DIR/help.txt"
tmux bind-key h display-popup -E -w 60% -h 40% "cat '$README_FILE'; echo ''; echo 'Press ANY KEY to close...'; read -n 1"

# tmux send-keys -t $SESSION:0.0 '' C-m
# tmux send-keys -t $SESSION:0.1 'git status' C-m

# Attach to session
tmux attach -t $SESSION
