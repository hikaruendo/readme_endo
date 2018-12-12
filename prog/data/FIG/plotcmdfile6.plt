set xlabel "time[*1.00 s]";set ylabel "alfa"
plot "../data/log/ALFA.dat" u 2:1 w l ti "alfa" ls 1
set terminal tgif
set output "../data/FIG/alfa.obj"
replot
set terminal x11
pause -1 "Press enter to quit. See ../data/FIG/plotcmdfile6.plt to replot alfa."
