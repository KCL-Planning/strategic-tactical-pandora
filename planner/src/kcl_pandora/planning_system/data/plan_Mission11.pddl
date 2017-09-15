Number of literals: 29
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_32) are preferable
Have identified that bigger values of (observed inspection_point_33) are preferable
Have identified that bigger values of (observed inspection_point_34) are preferable
Have identified that bigger values of (observed inspection_point_35) are preferable
Have identified that bigger values of (observed inspection_point_36) are preferable
Have identified that bigger values of (observed inspection_point_37) are preferable
Have identified that bigger values of (observed inspection_point_38) are preferable
Have identified that bigger values of (observed inspection_point_39) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 34.000
b (33.000 | 3.000)b (32.000 | 38.172)b (31.000 | 41.173)b (30.000 | 51.174)b (29.000 | 54.175)b (28.000 | 62.661)b (27.000 | 65.662)b (26.000 | 75.663)b (25.000 | 78.664)b (24.000 | 87.151)b (23.000 | 90.152)b (22.000 | 100.153)b (21.000 | 103.154)b (20.000 | 111.640)b (19.000 | 114.641)b (18.000 | 124.642)b (17.000 | 127.643)b (16.000 | 137.493)b (15.000 | 140.494)b (14.000 | 150.495)b (13.000 | 153.496)b (12.000 | 161.982)b (11.000 | 164.983)b (10.000 | 174.984)b (9.000 | 177.985)b (8.000 | 186.471)b (7.000 | 189.472)b (6.000 | 199.473)b (5.000 | 202.474)b (4.000 | 210.961)b (3.000 | 213.962)b (2.000 | 223.963)b (1.000 | 226.964)
; Plan found with metric 273.019
; States evaluated so far: 71
; Time 0.06
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [35.171]
38.173: (correct_position auv0 strategic_location_1)  [3.000]
41.174: (observe_inspection_point auv0 strategic_location_1 inspection_point_32)  [10.000]
51.175: (correct_position auv0 strategic_location_1)  [3.000]
54.176: (do_hover_fast auv0 strategic_location_1 strategic_location_2)  [8.485]
62.662: (correct_position auv0 strategic_location_2)  [3.000]
65.663: (observe_inspection_point auv0 strategic_location_2 inspection_point_33)  [10.000]
75.664: (correct_position auv0 strategic_location_2)  [3.000]
78.665: (do_hover_fast auv0 strategic_location_2 strategic_location_3)  [8.485]
87.152: (correct_position auv0 strategic_location_3)  [3.000]
90.153: (observe_inspection_point auv0 strategic_location_3 inspection_point_34)  [10.000]
100.154: (correct_position auv0 strategic_location_3)  [3.000]
103.155: (do_hover_fast auv0 strategic_location_3 strategic_location_4)  [8.485]
111.641: (correct_position auv0 strategic_location_4)  [3.000]
114.642: (observe_inspection_point auv0 strategic_location_4 inspection_point_35)  [10.000]
124.643: (correct_position auv0 strategic_location_4)  [3.000]
127.644: (do_hover_fast auv0 strategic_location_4 strategic_location_5)  [9.849]
137.494: (correct_position auv0 strategic_location_5)  [3.000]
140.495: (observe_inspection_point auv0 strategic_location_5 inspection_point_36)  [10.000]
150.496: (correct_position auv0 strategic_location_5)  [3.000]
153.497: (do_hover_fast auv0 strategic_location_5 strategic_location_6)  [8.485]
161.983: (correct_position auv0 strategic_location_6)  [3.000]
164.984: (observe_inspection_point auv0 strategic_location_6 inspection_point_37)  [10.000]
174.985: (correct_position auv0 strategic_location_6)  [3.000]
177.986: (do_hover_fast auv0 strategic_location_6 strategic_location_7)  [8.485]
186.472: (correct_position auv0 strategic_location_7)  [3.000]
189.473: (observe_inspection_point auv0 strategic_location_7 inspection_point_38)  [10.000]
199.474: (correct_position auv0 strategic_location_7)  [3.000]
202.475: (do_hover_fast auv0 strategic_location_7 strategic_location_8)  [8.485]
210.962: (correct_position auv0 strategic_location_8)  [3.000]
213.963: (observe_inspection_point auv0 strategic_location_8 inspection_point_39)  [10.000]
223.964: (correct_position auv0 strategic_location_8)  [3.000]
226.965: (do_hover_fast auv0 strategic_location_8 mission_site_start_point_1)  [46.054]

 * All goal deadlines now no later than 273.019

Resorting to best-first search
b (33.000 | 3.000)b (32.000 | 38.172)b (32.000 | 36.780)b (32.000 | 35.803)b (31.000 | 73.343)b (31.000 | 70.558)b (31.000 | 68.606)b (30.000 | 78.607)b (29.000 | 81.608)b (28.000 | 91.458)b (28.000 | 86.609)b (27.000 | 101.307)b (27.000 | 91.609)b (26.000 | 101.610)b (25.000 | 104.611)b (24.000 | 113.097)b (23.000 | 121.582)b (22.000 | 131.583)b (21.000 | 134.584)b (20.000 | 146.585)b (20.000 | 143.071)b (20.000 | 139.585)b (19.000 | 158.585)b (19.000 | 151.556)b (19.000 | 144.585)b (18.000 | 154.586)b (17.000 | 157.587)b (16.000 | 170.588)b (16.000 | 167.437)b (16.000 | 166.074)b (15.000 | 183.588)b (15.000 | 177.286)b (15.000 | 174.559)b (14.000 | 184.560)b (13.000 | 187.561)b (12.000 | 197.411)b (12.000 | 192.562)b (11.000 | 207.260)b (11.000 | 197.562)b (10.000 | 207.563)b (9.000 | 210.564)b (8.000 | 219.050)b (7.000 | 227.535)b (7.000 | 222.051)