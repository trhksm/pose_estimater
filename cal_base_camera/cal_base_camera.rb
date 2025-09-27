
class Qua
  attr_accessor :a, :b, :c, :d

  def initialize(a, b, c, d)
    @a, @b, @c, @d = a, b, c, d
  end

  def show
    printf("%+8.3f    %+8.3f %+8.3f %+8.3f\n", @a, @b, @c, @d)
  end

  def +(other)
    Qua.new(@a + other.a, @b + other.b, @c + other.c, @d + other.d)
  end

  def bar
    Qua.new(@a, -@b, -@c, -@d)
  end

  def *(other)
    a1, b1, c1, d1 = @a, @b, @c, @d
    a2, b2, c2, d2 = other.a, other.b, other.c, other.d
    Qua.new(
      a1*a2 - b1*b2 - c1*c2 - d1*d2,  # 1成分
      b1*a2 + a1*b2 - d1*c2 + c1*d2,  # i成分
      c1*a2 + d1*b2 + a1*c2 - b1*d2,  # j成分
      d1*a2 - c1*b2 + b1*c2 + a1*d2   # k成分
    )
  end

  def norm
    Math.sqrt(@a*@a + @b*@b + @c*@c + @d*@d)
  end

  def normalize
    n = norm
    raise "Error: normalize() norm is almost 0" if n < 1e-10
    inv = 1.0 / n
    Qua.new(@a*inv, @b*inv, @c*inv, @d*inv)
  end

  def self.from_axis_angle(rad, u)
    u = u.normalize
    Qua.new(
      Math.cos(rad/2.0),
      Math.sin(rad/2.0)*u.b,
      Math.sin(rad/2.0)*u.c,
      Math.sin(rad/2.0)*u.d
    )
  end
end

def rot0(p0, axis, rad)
  u = Qua.new(0, axis[0], axis[1], axis[2]).normalize
  src = Qua.new(0, p0[0], p0[1], p0[2])
  q = Qua.from_axis_angle(rad, u)
  qa = q * src
  dst = qa * q.bar
  [dst.b, dst.c, dst.d]
end

def rot(p0, axis, ofs, rad)
  org = [p0[0] - ofs[0], p0[1] - ofs[1], p0[2] - ofs[2]]
  rotated = rot0(org, axis, rad)
  [rotated[0] + ofs[0], rotated[1] + ofs[1], rotated[2] + ofs[2]]
end

def v3add(a, b, out)
  out[0] = a[0] + b[0]
  out[1] = a[1] + b[1]
  out[2] = a[2] + b[2]
end

def v3sub(a, b, out)
  out[0] = a[0] - b[0]
  out[1] = a[1] - b[1]
  out[2] = a[2] - b[2]
end

def v3mul(c, a, out)
  out[0] = c * a[0]
  out[1] = c * a[1]
  out[2] = c * a[2]
end

def v3len(a)
  Math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
end

def v3nom(a, out)
  length = v3len(a)
  if length == 0.0
    puts "error: v3nom input vector has length 0"
    out[0] = out[1] = out[2] = 0.0
  else
    out[0] = a[0] / length
    out[1] = a[1] / length
    out[2] = a[2] / length
  end
end

def v3dot(a, b)
  a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
end

def v3crs(a, b, out)
  out[0] = a[1]*b[2] - a[2]*b[1]
  out[1] = a[2]*b[0] - a[0]*b[2]
  out[2] = a[0]*b[1] - a[1]*b[0]
end

def distance_surface(oa, n, op)
  ap = []
  v3sub(op,oa,ap)
  return v3dot(ap, n)        #h n方向を正とした距離
end
#A = P   の時 AP = 0 h = 0     OK
#Aが平面上の時 APとnが垂直 h = 0 OK 
#A = O   の時 APとnが垂直 h = 0 OK
#P = O   の時 AP//n h = |AP|   OK
#oa opの向きのみ注意

def foot_perpendicular(oa, n, op, oq)
  pq = []
  h = distance_surface(oa, n, op)
  v3mul(-1.0 * h, n, pq)
  v3add(op, pq, oq)
end
#平面のn方向側にPがない時        OK

def line_plane_intersection(oa,n,op,l,ox) #面上一点の座標,面の方線,直線上一点の座標,直線の方向ベクトル,出力
  oq = []
  px = []
  foot_perpendicular(oa,n,op,oq)
  h = distance_surface(oa,n,op)
  cos_n_l = v3dot(n,l)
  if cos_n_l == 0.0
    puts "error: Since l and n are parallel, there is no inter section point"
  else
    v3mul(-1.0 * h.to_f / cos_n_l, l, px)
    v3add(op, px, ox)
  end
end

#Pが平面上 の時 h = 0 px = 0 ox = op  OK
#P = O    の時 op = ox = px = 0      OK
#nとlが垂直の時                       例外処理(交点を持たない)
#nとlが平行の時 ox = 0                OK
#nがP側 lが平面側           の時      OK
#nがPの反対側 lが平面側      の時      OK
#nがP側 lが平面の反対側      の時      OK
#nがPの反対側 lが平面の反対側 の時      OK

#変更箇所
height_camera = 0.9     #天井からカメラの距離[m]
height_base   = 3.0     #天井からベース(機体のずれが生じている場所)の距離[m] 
deg_fovh = 81.0         #カメラ視野角(水平)
deg_fovv = 40.0         #カメラ視野角(垂直)
ratioh = 0.2            #対象点の画面内における水平割合(右上基準)
ratiov = 0.3            #対象点の画面内における垂直割合(右上基準)
case_camera = 1         #0はベクトルでカメラ方向指定,elseは方位角・天頂角でカメラ方向指定

camera = [0.3,0.1,5.0]  #カメラの方向ベクトル
base = [0.3,0.1,5.0]    #ベースの方向ベクトル

deg_azimuth = 0.0       #方位角(カメラのずれ)x軸正の方向基準で反時計回り
deg_zenith  = 1.0       #天頂角(カメラのずれ)
deg_base_azimuth = 0.0  #方位角(カメラのずれ)x軸生の方向基準で反時計回り
deg_base_zenith = 1.0   #天頂角(カメラのずれ)
#変更箇所 終了
  

rad_fovh = deg_fovh * Math::PI / 180
rad_fovv = deg_fovv * Math::PI / 180

axis_x = [1.0,0.0,0.0]
axis_y = [0.0,1.0,0.0]
axis_z = [0.0,0.0,-1.0]
ofs_camera = [0.0,0.0, -1 * height_camera]          #カメラ位置ベクトル
ofs_base   = [0.0,0.0, -1 * height_base]            #ベース位置ベクトル
oa  = [0.0,0.0,0.0]                                 #天井の一点

#カメラ・ベースの方向ベクトル算出
if case_camera == 0 #ベクトルで指定
  camera_vec = []
  base_vec   = []
  v3nom(camera,camera_vec)
  v3nom(base,base_vec)

else                #角度で指定
  rad_azimuth      = deg_azimuth * Math::PI / 180.0 
  rad_zenith       = deg_zenith  * Math::PI / 180.0
  rad_base_azimuth = deg_base_azimuth * Math::PI / 180.0
  rad_base_zenith  = deg_base_zenith  * Math::PI / 180.0

  #baseのずれ方向ベクトル(方位角と天頂角で指定する場合)
  base_vec_x = Math.sin(rad_base_zenith) * Math.cos(rad_base_azimuth)
  base_vec_y = Math.sin(rad_base_zenith) * Math.sin(rad_base_azimuth)
  base_vec_z = Math.cos(rad_base_zenith) * -1                #z軸奥行きのため-
  base_vec   = [base_vec_x, base_vec_y ,base_vec_z]

  #カメラの視点方向ベクトル(方位角と天頂角で指定する場合)
  camera_vec_x = Math.sin(rad_zenith) * Math.cos(rad_azimuth)
  camera_vec_y = Math.sin(rad_zenith) * Math.sin(rad_azimuth)
  camera_vec_z = Math.cos(rad_zenith) * -1                    #z軸奥行きのため-
  camera_vec   = [camera_vec_x, camera_vec_y ,camera_vec_z]
end

#origin点算出
left_fov_origin_vec  = rot(axis_z, axis_y, oa, -1.0 * rad_fovh / 2.0)
right_fov_origin_vec = rot(axis_z, axis_y, oa, rad_fovh / 2.0)

leftup_fov_origin_vec     = rot(left_fov_origin_vec , axis_x, oa, -1.0 * rad_fovv / 2.0)
leftdown_fov_origin_vec   = rot(left_fov_origin_vec , axis_x, oa, rad_fovv / 2.0)
rightup_fov_origin_vec    = rot(right_fov_origin_vec, axis_x, oa, -1.0 * rad_fovv / 2.0)
rightdown_fov_origin_vec  = rot(right_fov_origin_vec, axis_x, oa, rad_fovv / 2.0)

leftup_fov_origin    = []
leftdown_fov_origin  = []
rightup_fov_origin   = []
rightdown_fov_origin = []

line_plane_intersection(oa,axis_z, ofs_camera, leftup_fov_origin_vec   , leftup_fov_origin)
line_plane_intersection(oa,axis_z, ofs_camera, leftdown_fov_origin_vec , leftdown_fov_origin)
line_plane_intersection(oa,axis_z, ofs_camera, rightup_fov_origin_vec  , rightup_fov_origin)
line_plane_intersection(oa,axis_z, ofs_camera, rightdown_fov_origin_vec, rightdown_fov_origin)


#baseずれ算出
base_v   = []
v3crs(base_vec,axis_z,base_v)
rad_base = Math.acos(v3dot(base_vec,axis_z))
ofs_camera = rot(ofs_camera,base_v,ofs_base, rad_base) #ここを無効化するとbaseのずれ考慮なしとなる


#cameraずれ算出
camera_v = []
v3crs(camera_vec, axis_z, camera_v)
rad_camera = Math.acos(v3dot(camera_vec,axis_z))

leftup_fov_vec    = rot(leftup_fov_origin_vec   , camera_v, oa, rad_camera)
leftdown_fov_vec  = rot(leftdown_fov_origin_vec , camera_v, oa, rad_camera)
rightup_fov_vec   = rot(rightup_fov_origin_vec  , camera_v, oa, rad_camera)
rightdown_fov_vec = rot(rightdown_fov_origin_vec, camera_v, oa, rad_camera)

leftup_fov    = []
leftdown_fov  = []
rightup_fov   = []
rightdown_fov = []

line_plane_intersection(oa, axis_z, ofs_camera, leftup_fov_vec   , leftup_fov)
line_plane_intersection(oa, axis_z, ofs_camera, leftdown_fov_vec , leftdown_fov)
line_plane_intersection(oa, axis_z, ofs_camera, rightup_fov_vec  , rightup_fov)
line_plane_intersection(oa, axis_z, ofs_camera, rightdown_fov_vec, rightdown_fov)


#対象点計算
point_origin = [0.0,0.0,0.0]
point        = [0.0,0.0,0.0]
a = [];b = [];c = [];d = []

v3mul(ratioh         * (1.0 - ratiov) , leftup_fov_origin   , a)
v3mul(ratioh         * ratiov         , leftdown_fov_origin , b)
v3mul((1.0 - ratioh) * (1.0 - ratiov) , rightup_fov_origin  , c)
v3mul((1.0 - ratioh) * ratiov         , rightdown_fov_origin, d)

v3add(point_origin, a, point_origin)
v3add(point_origin, b, point_origin)
v3add(point_origin, c, point_origin)
v3add(point_origin, d, point_origin)

v3mul(ratioh         * (1.0 - ratiov) , leftup_fov   , a)
v3mul(ratioh         * ratiov         , leftdown_fov , b)
v3mul((1.0 - ratioh) * (1.0 - ratiov) , rightup_fov  , c)
v3mul((1.0 - ratioh) * ratiov         , rightdown_fov, d)

v3add(point,a,point)
v3add(point,b,point)
v3add(point,c,point)
v3add(point,d,point)

#出力

File.open("fov_origin.txt", "w") do |f|
  f.puts "#{leftup_fov_origin[0]} #{leftup_fov_origin[1]}"
  f.puts "#{leftdown_fov_origin[0]} #{leftdown_fov_origin[1]}"
  f.puts "#{rightdown_fov_origin[0]} #{rightdown_fov_origin[1]}"
  f.puts "#{rightup_fov_origin[0]} #{rightup_fov_origin[1]}"
  f.puts "#{leftup_fov_origin[0]} #{leftup_fov_origin[1]}"
end

File.open("fov_points.txt", "w") do |f|
  f.puts "#{leftup_fov[0]} #{leftup_fov[1]}"
  f.puts "#{leftdown_fov[0]} #{leftdown_fov[1]}"
  f.puts "#{rightdown_fov[0]} #{rightdown_fov[1]}"
  f.puts "#{rightup_fov[0]} #{rightup_fov[1]}"
  f.puts "#{leftup_fov[0]} #{leftup_fov[1]}"
end

File.open("point_origin.txt", "w") do |f|
  f.puts "#{point_origin[0]} #{point_origin[1]}"
end

File.open("point.txt", "w") do |f|
  f.puts "#{point[0]} #{point[1]}"
end

File.open("point_camera.txt", "w") do |f|
  f.puts "#{ofs_camera[0]} #{ofs_camera[1]}"
end

=begin
puts "左上座標 x y [m]"
puts leftup_fov
puts "左下座標 x y [m]"
puts leftdown_fov
puts "右上座標 x y [m]"
puts rightup_fov
puts "右下座標 x y [m]"
puts rightdown_fov

puts "左上ずれ x y [m]"
puts leftup_fov[0] - leftup_fov_origin[0]
puts leftup_fov[1] - leftup_fov_origin[1]

puts "左下ずれ x y [m]"
puts leftdown_fov[0] - leftdown_fov_origin[0]
puts leftdown_fov[1] - leftdown_fov_origin[1]

puts "右上ずれ x y [m]"
puts rightup_fov[0] - rightup_fov_origin[0]
puts rightup_fov[1] - rightup_fov_origin[1]

puts "右下ずれ x y [m]"
puts rightdown_fov[0] - rightdown_fov_origin[0]
puts rightdown_fov[1] - rightdown_fov_origin[1]

puts "角度ずれ camera base [deg]"
puts rad_camera / Math::PI * 180.0
puts rad_base / Math::PI * 180.0

=end

puts "対象点ずれ x y [m]"
puts point[0] - point_origin[0]
puts point[1] - point_origin[1]

p `gnuplot plot_fov.gp`
