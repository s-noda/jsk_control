#!/usr/bin/env sh

LATEST_FILE="`pwd`/graphviz/`ls -lt graphviz | head -n 2 | tail -n 1 | gawk '{print $9}'`"
## echo $LATEST_FILE;
cd $LATEST_FILE;
