set xlabel "time[*1.00 s]";set ylabel "ESS"
plot "../data/log/ESS.dat" u 1:2 w l ti "ESS" ls 6
set terminal tgif
set output "../data/FIG/ESS.obj"
replot
set terminal x11
pause -1 "Press enter to quit. See ../data/FIG/plotcmdfile2.plt to replot ESS."
