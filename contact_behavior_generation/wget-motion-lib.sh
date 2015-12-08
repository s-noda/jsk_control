#!/usr/bin/env bash

wget www.jsk.t.u-tokyo.ac.jp/~s-noda/motion-lib-proposal.l ;
mkdir -p euslisp/euslib/irteus_proposals;
mv motion-lib-proposal.l euslisp/euslib/irteus_proposals;
