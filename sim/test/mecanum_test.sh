#!/usr/bin/env bash
TOPIC=/cmd_vel          
RATE=1                               

step () { gz topic -t $TOPIC -m gz.msgs.Twist -p "$1" ; sleep $RATE ; }

# forward 2 s
for i in {1..20};  do step "linear:{x:0.5}" ; done
# strafe left 2 s
for i in {1..20};  do step "linear:{y:0.5}" ; done
# diagonal forward-right 2 s
for i in {1..20};  do step "linear:{x:0.4,y:-0.4}" ; done
# spin in place CCW 2 s
for i in {1..20};  do step "angular:{z:1.0}" ; done
# stop
step "{}"