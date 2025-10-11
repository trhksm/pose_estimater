set title "3D Vectors Visualization"
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set grid
set view 30, 30
set key outside

# 3Dプロットの範囲調整（必要に応じて）
set xrange [-2:2]
set yrange [0:2]
set zrange [-2:2]

# splot で複数のベクトルファイルを描画
splot \
    "ideal_aruco_unit_vecs.txt" using 1:2:3:4:5:6 with vectors nohead lc rgb "blue" title "Unit Vectors", \
    "ideal_aruco_positions.txt" using 1:2:3:4:5:6 with vectors nohead lc rgb "green" title "Aruco Positions", \
    "ideal_fov_positions.txt" using 1:2:3:4:5:6 with vectors nohead lc rgb "red" title "FOV Positions", \
    "camera_rotate_axis.txt" using 1:2:3:4:5:6 with vectors nohead lc rgb "orange" title "Rotate Axis"

