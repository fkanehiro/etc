set term postscript landscape enhanced color "Times-Roman" 24 
set size 1.0,0.8
set output "rightdist.eps"
set key right top

set xlabel "Time[s]"
set ylabel "Minimum Distance[m]"
plot "orig/const.dat" u 1:3 w l lw 5 t "initial trajectory", "full/const.dat" u 1:3 w l lw 5 t "executed trajectory"

