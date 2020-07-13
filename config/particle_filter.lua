map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

if (false)
then
-- 2019-12-27-14-25-29_GDC3_AI_offices.bag
  init_x = 101.378
  init_y = 61
  init_r = 0
end

if (false)
then
-- 2019-12-27-15-27-33_GDC1_north.bag
  map = "maps/GDC1.txt"
  init_x = 40.32
  init_y = 71.58
  init_r = 180
end

if (false)
then
-- 2019-12-27-15-16-21_GDC2.bag
  map = "maps/GDC2.txt"
  init_x = 53.32
  init_y = 42.58
  init_r = 0
end



k1 = 0.5;  -- Angular error per unit rotation (radians / radian)
k2 = 0.2;  -- Angular error per unit translation (radians / meter)
k3 = 0.4;  -- Translation error per unit rotation (meters / radian)
k4 = 0.3;  -- Translation error per unit translation (meters / meter)

laser_stddev = 0.5;
laser_short_cutoff = 0.5;
laser_long_cutoff = 100.5;
laser_range_limit = 7;
