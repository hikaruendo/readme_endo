set xlabel "time[*1.00 s]";set ylabel "orientation[deg]"
plot "../data/position_usingPF.dat" u 4:3 w l ti "usingPF" ls 1, "../data/position_onlyU.dat" u 4:3 w l ti "Inputonly" ls 3, "../data/log/ORS.dat" u 2:1 w l ti "ORS" ls 4,"../data/log/KINECT.dat" u 4:3 w l ti "KINECT" ls 1
set terminal tgif
set output "../data/FIG/STATE_ANGLE.obj"
replot
set terminal x11
pause -1 "Press enter to quit. See ../data/FIG/plotcmdfile1.plt to replot STATE_ANGLE."
