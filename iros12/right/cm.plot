set term postscript landscape enhanced color "Times-Roman" 24 
set size 1.0,0.8
set output "rightcm.eps"
set key left top

set xlabel "Time[s]"
set ylabel "ZMP[m]"
plot "orig/com.dat" u 1:2 w l lw 5 t "X(initial trajectory)", "woca/com.dat" u 1:2 w l lw 5 t "X(executed trajectory)", "orig/com.dat" u 1:3 w l lw 5 t "Y(initial trajectory)", "woca/com.dat" u 1:3 w l lw 5 t "Y(executed trajectory)"

