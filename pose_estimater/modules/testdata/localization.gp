set title "3D Vectors Visualization for localization"
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
    "localization/ideal_fov_unit_vecs.txt" using 1:2:3:4:5:6 with vectors nohead lc rgb "blue" title "ideal fov unit vectors", \
    "localization/fov_vecs.txt" using 1:2:3:($4-$1):($5-$2):($6-$3) with vectors nohead lc rgb "green" title "fov vectors", \
    "localization/camera_to_aruco_vecs.txt" using 1:2:3:($4-$1):($5-$2):($6-$3) with vectors nohead lc rgb "red" title "camera to aruco vecs", \
    "localization/camera_world_positions.txt" using 1:2:3:4:5:6 with vectors nohead lc rgb "orange" title "camera world positions"

pause -1 "Press Enter to close"
