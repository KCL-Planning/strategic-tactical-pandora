Number of literals: 29
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_40) are preferable
Have identified that bigger values of (observed inspection_point_41) are preferable
Have identified that bigger values of (observed inspection_point_42) are preferable
Have identified that bigger values of (observed inspection_point_43) are preferable
Have identified that bigger values of (observed inspection_point_44) are preferable
Have identified that bigger values of (observed inspection_point_45) are preferable
Have identified that bigger values of (observed inspection_point_46) are preferable
Have identified that bigger values of (observed inspection_point_47) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 34.000
b (33.000 | 3.000)b (32.000 | 74.254)b (31.000 | 77.255)b (30.000 | 87.256)b (29.000 | 90.257)b (28.000 | 98.743)b (27.000 | 101.744)b (26.000 | 111.745)b (25.000 | 114.746)b (24.000 | 123.233)b (23.000 | 126.234)b (22.000 | 136.235)b (21.000 | 139.236)b (20.000 | 147.722)b (19.000 | 150.723)b (18.000 | 160.724)b (17.000 | 163.725)b (16.000 | 173.575)b (15.000 | 176.576)b (14.000 | 186.577)b (13.000 | 189.578)b (12.000 | 198.064)b (11.000 | 201.065)b (10.000 | 211.066)b (9.000 | 214.067)b (8.000 | 222.553)b (7.000 | 225.554)b (6.000 | 235.555)b (5.000 | 238.556)b (4.000 | 247.043)b (3.000 | 250.044)b (2.000 | 260.045)b (1.000 | 263.046)
; Plan found with metric 330.995
; States evaluated so far: 71
; Time 0.06
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [71.253]
74.255: (correct_position auv0 strategic_location_1)  [3.000]
77.256: (observe_inspection_point auv0 strategic_location_1 inspection_point_40)  [10.000]
87.257: (correct_position auv0 strategic_location_1)  [3.000]
90.258: (do_hover_fast auv0 strategic_location_1 strategic_location_2)  [8.485]
98.744: (correct_position auv0 strategic_location_2)  [3.000]
101.745: (observe_inspection_point auv0 strategic_location_2 inspection_point_41)  [10.000]
111.746: (correct_position auv0 strategic_location_2)  [3.000]
114.747: (do_hover_fast auv0 strategic_location_2 strategic_location_3)  [8.485]
123.234: (correct_position auv0 strategic_location_3)  [3.000]
126.235: (observe_inspection_point auv0 strategic_location_3 inspection_point_42)  [10.000]
136.236: (correct_position auv0 strategic_location_3)  [3.000]
139.237: (do_hover_fast auv0 strategic_location_3 strategic_location_4)  [8.485]
147.723: (correct_position auv0 strategic_location_4)  [3.000]
150.724: (observe_inspection_point auv0 strategic_location_4 inspection_point_43)  [10.000]
160.725: (correct_position auv0 strategic_location_4)  [3.000]
163.726: (do_hover_fast auv0 strategic_location_4 strategic_location_5)  [9.849]
173.576: (correct_position auv0 strategic_location_5)  [3.000]
176.577: (observe_inspection_point auv0 strategic_location_5 inspection_point_44)  [10.000]
186.578: (correct_position auv0 strategic_location_5)  [3.000]
189.579: (do_hover_fast auv0 strategic_location_5 strategic_location_6)  [8.485]
198.065: (correct_position auv0 strategic_location_6)  [3.000]
201.066: (observe_inspection_point auv0 strategic_location_6 inspection_point_45)  [10.000]
211.067: (correct_position auv0 strategic_location_6)  [3.000]
214.068: (do_hover_fast auv0 strategic_location_6 strategic_location_7)  [8.485]
222.554: (correct_position auv0 strategic_location_7)  [3.000]
225.555: (observe_inspection_point auv0 strategic_location_7 inspection_point_46)  [10.000]
235.556: (correct_position auv0 strategic_location_7)  [3.000]
238.557: (do_hover_fast auv0 strategic_location_7 strategic_location_8)  [8.485]
247.044: (correct_position auv0 strategic_location_8)  [3.000]
250.045: (observe_inspection_point auv0 strategic_location_8 inspection_point_47)  [10.000]
260.046: (correct_position auv0 strategic_location_8)  [3.000]
263.047: (do_hover_fast auv0 strategic_location_8 mission_site_start_point_1)  [67.949]

 * All goal deadlines now no later than 330.995

Resorting to best-first search
b (33.000 | 3.000)b (32.000 | 74.254)b (32.000 | 73.797)b (31.000 | 145.507)b (31.000 | 144.592)b (30.000 | 154.593)b (29.000 | 157.594)b (28.000 | 162.595)b (27.000 | 167.595)b (26.000 | 177.596)b (25.000 | 180.597)b (24.000 | 189.083)b (23.000 | 197.569)b (22.000 | 207.570)b (21.000 | 210.571)b (20.000 | 219.057)b (20.000 | 215.572)b (19.000 | 227.542)b (19.000 | 220.572)b (18.000 | 230.573)b (17.000 | 233.574)b (16.000 | 243.423)b (16.000 | 242.060)b (15.000 | 253.272)b (15.000 | 250.545)b (15.000 | 245.061)