set xlabel "x[m]";set ylabel "y[m]"
set size ratio -1
plot "../data/log/GPS.dat" u ($1):($2) w l ti "GPS" ls 7 lw 2, "../data/position_onlyU.dat" u ($1/1000):($2/1000) w l ti "Inputonly" ls 3 lw 2, "../data/log/TRUE.dat" u ($1):($2) w l ti "TRUE" ls 2 lw 2, "../data/position_usingPF.dat" u ($1/1000):($2/1000) w l ti "usingPF" ls 5 lw 2,"../data/log/KINECT.dat" u ($1):($2) w l ti "KINECT" ls 1 lw 2
set terminal tgif
set output "../data/FIG/STATE_ANGLE.obj"
replot
set terminal x11
pause -1 "Press enter to quit. See ../data/FIG/plotcmdfile0.plt to replot STATE_ANGLE."
