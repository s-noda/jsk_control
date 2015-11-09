#!/usr/bin/env bash

cp horizontail_dtau TorqueGradient;
cp horizontail_df PseudoGradient;
gnuplot <<EOF
set terminal postscript eps color enhanced
set tics font "Times New Roman,25"   # 目盛りのフォントの変更
set xlabel font "Times New Roman,25" # xlabelのフォントの変更
set ylabel font "Times New Roman,25" # ylabelのフォントの変更
set zlabel font "Times New Roman,25" # zlabelのフォントの変更
set key font "Times New Roman,25"    # 凡例のフォントの変更
set output "horizontal.eps"
set grid
set size ratio 0.5
set xlabel "STEP"
set ylabel "||Joint Torque/Torque Max||"
set title "_"
set yrange [ 0.1 : 0.3 ]
set mxtics 5
set mytics 5
set xtics 10
set ytics 0.1
set key spacing 4
plot 'TorqueGradient' with line linewidth 10, 'PseudoGradient' with line linewidth 10
EOF

cp slop15_dtau TorqueGradient;
cp slop15_df PseudoGradient;
gnuplot <<EOF
set terminal postscript eps color enhanced
set tics font "Times New Roman,25"   # 目盛りのフォントの変更
set xlabel font "Times New Roman,25" # xlabelのフォントの変更
set ylabel font "Times New Roman,25" # ylabelのフォントの変更
set zlabel font "Times New Roman,25" # zlabelのフォントの変更
set key font "Times New Roman,25"    # 凡例のフォントの変更
set output "slope15.eps"
set grid
set size ratio 0.5
set xlabel "STEP"
set ylabel "||Joint Torque/Torque Max||"
set title "_"
set yrange [ 0.2 : 0.6 ]
set mxtics 5
set mytics 5
set xtics 10
set ytics 0.1
set key spacing 4
plot 'TorqueGradient' with line linewidth 10, 'PseudoGradient' with line linewidth 10
EOF
