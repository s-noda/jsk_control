#!/usr/bin/env bash

CMD="rosbag record " ;
BLACKLIST=blacklist.topic ;
BLL="";

function gen_blacklist(){
    echo -n "" > $BLACKLIST;
    for topic in `rostopic list`;
    do
        echo -n $topic "-->" ;
        ## if [ "`rostopic info $topic | grep -e \"Image\|image\|Cloud\|cloud\|pcl\|visual\|depth\|multisense\"`" ];
        if [ "`echo $topic | grep -e \"Image\|image\|Cloud\|cloud\|pcl\|visual\|depth\|multisense\|laser\"`" ];
        then
            echo "$topic" >> $BLACKLIST;
            echo -e "\e[31mblack\e[m" ;
        else
            echo -e "\e[32mwhite\e[m" ;
        fi ;
    done;
}
if [ ! -e "$BLACKLIST" ];
then
    echo -e "\e[31mgenerate black list\e[m";
    gen_blacklist;
fi

echo -e "\e[31mwaiting rosbag record ...\e[m";
for topic in `rostopic list`;
do
    ## echo $topic ;
    if [ "`grep -e \"^$topic$\" $BLACKLIST`" ];
    then
        BLL="$BLL $topic";
        echo -e "$topic --> \e[31mblack\e[m" ;
    else
        CMD="$CMD $topic";
        echo -e "$topic --> \e[32mwhite\e[m" ;
    fi ;
done;
echo -e "\e[31m$BLL\e[m";
echo -e "\e[32m$CMD\e[m";
echo $CMD  | /bin/sh ;
