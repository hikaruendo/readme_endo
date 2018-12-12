set xlabel "time[*1.00 s]";set ylabel "true_dis[m]"
plot "../data/log/TRUE_DIS.dat" u 3:1 w l ti "true_dis" ls 1
set terminal tgif
set output "../data/FIG/true_dis.obj"
replot
set terminal x11
pause -1 "Press enter to quit. See ../data/FIG/plotcmdfile7.plt to replot true_dis."
