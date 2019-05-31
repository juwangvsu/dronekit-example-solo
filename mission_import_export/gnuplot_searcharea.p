   set   autoscale                        # scale axes automatically
      unset log                              # remove any log-scaling
      unset label                            # remove any previous labels
      set xtic auto                          # set xtics automatically
      set ytic auto                          # set ytics automatically
      set title "2tie UAV search area"
      set xlabel "x (meters)"
      set ylabel "y (meters)"
      set arrow from 0.45,3 to 0.6, 2.6
      set xr [-200:200]
      set yr [-200:200]
# set term postscript eps color blacktext "Helvetica" 24
# set output "searcharea.eps"
      plot    "mpmission_local.txt" using 3:4 title '2 UAVs' with linespoints

pause -1  "Hit return to continue"

