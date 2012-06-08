set term postscript landscape enhanced color "Times-Roman" 24 
set size 1.0,0.8
set output "rightconsts.eps"

set xlabel "Time[s]"
set ylabel "Number of spheres"
plot "full/const.dat" u 1:2 w l lw 5 t ""

