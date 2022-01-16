# clear
set term wxt size 1280,960
set terminal wxt font "Arial,21"
set multiplot layout 2,1
set xrange[0:1.1]
set xlabel "x axis"
set ylabel "y axis"
set yrange[-0.14:0.1]
#set zlabel "z axis"
#set zrange[0:10]
plot "position.dat" using 1:2 title "CoM position" with linespoints ps 1 pt 7,"position.dat" using 3:4 title "CP position" with linespoints ps 1 pt 6 ,"position.dat" using 5:6 title "zmp position" with linespoints ps 3 pt 14
set xrange[0:4]
set yrange[-0.4:1.0]
set xlabel "t[s] axis"
plot "velo.dat" using 1:2 title "CoM x velocity " with linespoints ps 1 pt 7, "velo.dat" using 1:3 title "CoM y velocity " with linespoints ps 1 pt 7
unset multiplot