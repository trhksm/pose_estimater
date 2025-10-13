set title "3D Vectors Visualization"
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set grid
set view 30, 30
set key outside
set xrange [-1:1]
set yrange [-1:1]
set zrange [-1:1]
splot \
    "ideal_aruco_unit_vecs.txt" using 1:2:3:4:5:6 with vectors nohead lc rgb "blue" title "Unit Vectors", \
    "ideal_aruco_positions.txt" using 1:2:3:($4-$1):($5-$2):($6-$3) with vectors nohead lc rgb "green" title "Aruco Positions", \
    "ideal_fov_positions.txt" using 1:2:3:($4-$1):($5-$2):($6-$3) with vectors nohead lc rgb "red" title "FOV Positions", \
    "camera_rotate_axis.txt" using 1:2:3:4:5:6 with vectors nohead lc rgb "orange" title "Rotate Axis"

pause -1 "Press Enter to close"
