#!/usr/bin/env sh

FILE_NAME="graphviz";
TYPE="eps";

if [ "$1" ];
then
    FILE_NAME=$1;
fi

if [ "$2" ];
then
    TYPE=$2;
fi

dot -T$TYPE $FILE_NAME.dot -o $FILE_NAME.$TYPE;
## neato -n -Gsplines=true -Goverlap=false -T$TYPE $FILE_NAME.dot -n -o $FILE_NAME.$TYPE;
## fdp
