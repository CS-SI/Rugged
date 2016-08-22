# gnuplot script file for plotting bandwidth over time
#!/usr/bin/gnuplot


set terminal pdf  size 3,2
set output 'atmospheric-deviation.pdf'
set grid

set title "Atmospheric Correction"
set xlabel "angle (radians)"
set ylabel "correction (meters)"

plot "atmospheric-deviation.csv" using 1:0 with lines title ""
