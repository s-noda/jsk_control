#!/usr/bin/env bash

gnuplot <<EOF
set terminal postscript eps color enhanced
set tics font "Times New Roman,25"   # 目盛りのフォントの変更
set xlabel font "Times New Roman,25" # xlabelのフォントの変更
set ylabel font "Times New Roman,25" # ylabelのフォントの変更
set zlabel font "Times New Roman,25" # zlabelのフォントの変更
set key font "Times New Roman,25"    # 凡例のフォントの変更
set key left top
set key width 8
set output "test_time_comb.eps"
set grid
set size ratio 0.5
set xlabel "Joint DOF"
set ylabel "Computation time [sec]"
set title "_"
set yrange [ 0.0 : 1.0 ]
set xrange [ 0.0 : 36.0 ]
set mxtics 5
set mytics 5
set xtics 10
set ytics 0.2
set key spacing 4
plot 'PseudoGradient(k=1)' with line linewidth 10, 'PseudoGradient(k=2)' with line linewidth 10, 'PseudoGradient(k=3)' with line linewidth 10, 'PseudoGradient(k=4)' with line linewidth 10, 'TorqueGradient(k=1)' with line linewidth 10, 'TorqueGradient(k=2)' with line linewidth 10, 'TorqueGradient(k=3)' with line linewidth 10, 'TorqueGradient(k=4)' with line linewidth 10
EOF

gnuplot <<EOF
set terminal postscript eps color enhanced
set tics font "Times New Roman,25"   # 目盛りのフォントの変更
set xlabel font "Times New Roman,25" # xlabelのフォントの変更
set ylabel font "Times New Roman,25" # ylabelのフォントの変更
set zlabel font "Times New Roman,25" # zlabelのフォントの変更
set key font "Times New Roman,25"    # 凡例のフォントの変更
set key left top
set key width 8
set output "test_time_zoom.eps"
set grid
set size ratio 0.5
set xlabel "Joint DOF"
set ylabel "Computation time [sec]"
set title "_"
set yrange [ 0.0 : 0.1 ]
set xrange [ 1.0 : 10.0 ]
set mxtics 5
set mytics 5
set xtics 10
set ytics 0.02
set key spacing 4
plot 'PseudoGradient(k=1)' with line linewidth 10, 'PseudoGradient(k=2)' with line linewidth 10, 'PseudoGradient(k=3)' with line linewidth 10, 'PseudoGradient(k=4)' with line linewidth 10, 'TorqueGradient(k=1)' with line linewidth 10, 'TorqueGradient(k=2)' with line linewidth 10, 'TorqueGradient(k=3)' with line linewidth 10, 'TorqueGradient(k=4)' with line linewidth 10
EOF

gnuplot <<EOF
set terminal postscript eps color enhanced
set tics font "Times New Roman,25"   # 目盛りのフォントの変更
set xlabel font "Times New Roman,25" # xlabelのフォントの変更
set ylabel font "Times New Roman,25" # ylabelのフォントの変更
set zlabel font "Times New Roman,25" # zlabelのフォントの変更
set key font "Times New Roman,25"    # 凡例のフォントの変更
set key left top
set key width 8
set output "test_time_pseudo.eps"
set grid
set size ratio 0.5
set xlabel "Joint DOF"
set ylabel "Computation time [sec]"
set title "_"
set yrange [ 0.0 : 0.05 ]
set xrange [ 1.0 : 36.0 ]
set mxtics 5
set mytics 5
set xtics 10
set ytics 0.01
set key spacing 4
plot 'PseudoGradient(k=1)' with line linewidth 10, 'PseudoGradient(k=2)' with line linewidth 10, 'PseudoGradient(k=3)' with line linewidth 10, 'PseudoGradient(k=4)' with line linewidth 10
EOF

gnuplot <<EOF
set terminal postscript eps color enhanced
set tics font "Times New Roman,25"   # 目盛りのフォントの変更
set xlabel font "Times New Roman,25" # xlabelのフォントの変更
set ylabel font "Times New Roman,25" # ylabelのフォントの変更
set zlabel font "Times New Roman,25" # zlabelのフォントの変更
set key font "Times New Roman,25"    # 凡例のフォントの変更
set key left top
set key width 8
set output "test_time_torque.eps"
set grid
set size ratio 0.5
set xlabel "Joint DOF"
set ylabel "Computation time [sec]"
set title "_"
set yrange [ 0.0 : 1.0 ]
set xrange [ 1.0 : 36.0 ]
set mxtics 5
set mytics 5
set xtics 10
set ytics 0.2
set key spacing 4
plot 'TorqueGradient(k=1)' with line linewidth 10, 'TorqueGradient(k=2)' with line linewidth 10, 'TorqueGradient(k=3)' with line linewidth 10, 'TorqueGradient(k=4)' with line linewidth 10
EOF

sed -i "s/^\%\%BoundingBox\: .\+$/%%BoundingBox: 50 80 410 272/g" *.eps;