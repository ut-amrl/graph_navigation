#!/bin/bash

tmux -V > /dev/null 2>&1
if [ $? -eq 127 ]
then
    echo "tmux is not installed! Cannot continue"
    exit 127
fi

session="ROS"

tmux has-session -t $session > /dev/null 2>&1
RUNNING=$?

ARG=$1
if [[ "${ARG}" == "stop" ]]
then
    if [ $RUNNING -ne 0 ]
    then
        echo "tmux session is not running."
        exit 0
    fi
    
    echo "Killing running tmux session"
    tmux kill-session -t $session > /dev/null 2>&1
    exit 0
fi

if [ $RUNNING -eq 0 ]
then
    echo "ERROR: tmux session is already running. Kill current session with ./tmux_session.sh stop"
    exit 127
fi


if [ "${ARG}" == "-d" ]
then
    INSTALL_PATH=$2
    if [ -z "${INSTALL_PATH}" ]
    then
        INSTALL_PATH="${HOME}/"
    fi

    # check for /  at the end
    if [ ${INSTALL_PATH: -1} != '/' ]
    then
        INSTALL_PATH="${INSTALL_PATH}/"
    fi
else
    INSTALL_PATH="${HOME}/"
fi

if ! [[ -d "${INSTALL_PATH}ut_automata" ]]
then
    echo "${INSTALL_PATH}ut_automata does not exist, please provide a valid path using ./tmux_session.sh -d <INSTALL_PATH>"
    exit 127
fi

tmux new-session -d -s $session 
tmux set mouse on
window=0

tmux rename-window -t $session:$window 'roscore'
tmux send-keys -t $session:$window 'roscore' C-m

window=1
tmux new-window -t $session:$window -n 'simulator'
tmux send-keys -t $session:$window "cd ${INSTALL_PATH}/ut_automata && ./bin/simulator --localize" C-m

window=2
tmux new-window -t $session:$window -n 'websocket'
tmux send-keys -t $session:$window "cd ${INSTALL_PATH}/ut_automata && ./bin/websocket" C-m

if ! [ -z ${CS393R_DOCKER_CONTEXT+x} ]
then
    /bin/bash
fi