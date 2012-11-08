set term postscript landscape enhanced color "Times-Roman" 24 
set size 1.0,0.8
set output "tm.eps"
set key left top

set xlabel "Time[s]"
set ylabel "Processing Time[ms]"
plot "full/tm.dat" u 1:(($2+$3)*1000) w l lw 5 t "total","full/tm.dat" u 1:($2*1000) w l lw 5 t "finding spheres","full/tm.dat" u 1:($3*1000) w l lw 5 t "solving IK"


