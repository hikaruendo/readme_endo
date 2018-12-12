set xlabel "time[*1.00 s]";set ylabel "StandardDeviation of x[m]"
plot "../data/log/VARIANCE.dat" u 1:(sqrt($2)/1000.) w l ti "STD" ls 1
set terminal tgif
set output "../data/FIG/VAR_X.obj"
replot
set terminal x11
pause -1 "Press enter to quit. See ../data/FIG/plotcmdfile3.plt to replot VAR_X."
