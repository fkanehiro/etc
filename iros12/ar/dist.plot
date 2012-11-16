set term postscript landscape enhanced color "Times-Roman" 24 
set size 1.0,0.8
set output "dist.eps"
set key left top

set xlabel "Time[s]"
set ylabel "Minimum Distance[m]"
plot [0:10] "woca/const.dat" u 1:3 w l lw 5 t "without collision avoidance", "full/const.dat" u 1:3 w l lw 5 t "with collision avoidance", 0.005 lw 5 notitle, 0.0 lw 5 notitle

