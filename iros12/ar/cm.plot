set term postscript landscape enhanced color "Times-Roman" 24 
set size 1.0,0.8
set output "cm.eps"
set key left top

set xlabel "Time[s]"
set ylabel "COG[m]"
plot [0:10] "orig/com.dat" u 1:2 w l lw 5 t "X(without compensation)", "full/com.dat" u 1:2 w l lw 5 t "X(with compensation)", "orig/com.dat" u 1:3 w l lw 5 t "Y(without compensation)", "full/com.dat" u 1:3 w l lw 5 t "Y(with compensation)"

