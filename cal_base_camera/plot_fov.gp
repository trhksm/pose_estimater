set terminal pngcairo size 800,600
set output "fov.png"

set title "Camera FOV Plot"
set xlabel "X [m]"
set ylabel "Y [m]"
set size ratio -1

plot "fov_origin.txt" with linespoints lw 2 lc rgb "blue" title "FOV origin", \
     "fov_points.txt" with linespoints lw 2 lc rgb "red"  title "FOV rotated", \
     "point_origin.txt" with points pt 7 ps 1.5 lc rgb "blue" title "Point origin", \
     "point.txt"        with points pt 7 ps 1.5 lc rgb "red"  title "Point", \
     "point_camera.txt" with points pt 7 ps 1.5 lc rgb "green" title "Point camera", \
     "-" with points pt 7 ps 2 lc rgb "black" title "Camera"
0 0
e

