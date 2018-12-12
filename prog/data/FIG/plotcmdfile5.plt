set xlabel "time[*1.00 s]";set ylabel "StandardDeviation of orientation[deg]"
plot "../data/log/VARIANCE.dat" u 1:(sqrt($4)) w l ti "VARIANCE" ls 1
set terminal tgif
set output "../data/FIG/VAR_THETA.obj"
replot
set terminal x11
pause -1 "Press enter to quit. See ../data/FIG/plotcmdfile5.plt to replot VAR_THETA."
