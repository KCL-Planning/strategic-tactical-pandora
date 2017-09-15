Number of literals: 29
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_24) are preferable
Have identified that bigger values of (observed inspection_point_25) are preferable
Have identified that bigger values of (observed inspection_point_26) are preferable
Have identified that bigger values of (observed inspection_point_27) are preferable
Have identified that bigger values of (observed inspection_point_28) are preferable
Have identified that bigger values of (observed inspection_point_29) are preferable
Have identified that bigger values of (observed inspection_point_30) are preferable
Have identified that bigger values of (observed inspection_point_31) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 34.000
b (33.000 | 3.000)b (30.000 | 67.515)b (29.000 | 70.516)b (28.000 | 80.517)b (27.000 | 83.518)b (26.000 | 101.519)b (25.000 | 104.520)b (24.000 | 114.521)b (23.000 | 117.522)b (22.000 | 131.524)b (21.000 | 134.525)b (20.000 | 152.965)b (19.000 | 155.966)b (18.000 | 165.967)b (17.000 | 168.968)b (16.000 | 172.969)b (15.000 | 175.970)b (14.000 | 185.971)b (13.000 | 188.972)b (12.000 | 207.412)b (11.000 | 210.413)b (10.000 | 220.414)b (9.000 | 223.415)b (8.000 | 238.121)b (7.000 | 241.122)b (6.000 | 251.123)b (5.000 | 254.124)b (4.000 | 274.125)b (3.000 | 277.126)b (2.000 | 287.127)b (1.000 | 290.128)
; Plan found with metric 327.335
; States evaluated so far: 74
; Time 0.06
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [64.514]
67.516: (correct_position auv0 strategic_location_1)  [3.000]
70.517: (observe_inspection_point auv0 strategic_location_1 inspection_point_24)  [10.000]
80.518: (correct_position auv0 strategic_location_1)  [3.000]
83.519: (do_hover_fast auv0 strategic_location_1 strategic_location_5)  [18.000]
101.520: (correct_position auv0 strategic_location_5)  [3.000]
104.521: (observe_inspection_point auv0 strategic_location_5 inspection_point_28)  [10.000]
114.522: (correct_position auv0 strategic_location_5)  [3.000]
117.523: (do_hover_controlled auv0 strategic_location_5 strategic_location_4)  [4.000]
121.524: (observe_inspection_point auv0 strategic_location_4 inspection_point_27)  [10.000]
131.525: (correct_position auv0 strategic_location_4)  [3.000]
134.526: (do_hover_fast auv0 strategic_location_4 strategic_location_2)  [18.439]
152.966: (correct_position auv0 strategic_location_2)  [3.000]
155.967: (observe_inspection_point auv0 strategic_location_2 inspection_point_25)  [10.000]
165.968: (correct_position auv0 strategic_location_2)  [3.000]
168.969: (do_hover_fast auv0 strategic_location_2 strategic_location_3)  [4.000]
172.970: (correct_position auv0 strategic_location_3)  [3.000]
175.971: (observe_inspection_point auv0 strategic_location_3 inspection_point_26)  [10.000]
185.972: (correct_position auv0 strategic_location_3)  [3.000]
188.973: (do_hover_fast auv0 strategic_location_3 strategic_location_6)  [18.439]
207.413: (correct_position auv0 strategic_location_6)  [3.000]
210.414: (observe_inspection_point auv0 strategic_location_6 inspection_point_29)  [10.000]
220.415: (correct_position auv0 strategic_location_6)  [3.000]
223.416: (do_hover_fast auv0 strategic_location_6 strategic_location_7)  [14.705]
238.122: (correct_position auv0 strategic_location_7)  [3.000]
241.123: (observe_inspection_point auv0 strategic_location_7 inspection_point_30)  [10.000]
251.124: (correct_position auv0 strategic_location_7)  [3.000]
254.125: (do_hover_fast auv0 strategic_location_7 strategic_location_8)  [20.000]
274.126: (correct_position auv0 strategic_location_8)  [3.000]
277.127: (observe_inspection_point auv0 strategic_location_8 inspection_point_31)  [10.000]
287.128: (correct_position auv0 strategic_location_8)  [3.000]
290.129: (do_hover_fast auv0 strategic_location_8 mission_site_start_point_1)  [37.205]

 * All goal deadlines now no later than 327.335

Resorting to best-first search
b (33.000 | 3.000)b (30.000 | 67.515)b (29.000 | 132.028)b (28.000 | 142.029)b (27.000 | 145.030)b (26.000 | 163.031)b (25.000 | 181.031)b (24.000 | 191.032)b (23.000 | 194.033)b (22.000 | 208.035)b (21.000 | 211.036)b (20.000 | 229.476)b (20.000 | 229.037)b (20.000 | 215.037)b (19.000 | 247.259)b (19.000 | 247.037)b (19.000 | 219.037)b (18.000 | 229.038)b (17.000 | 232.039)b (16.000 | 250.040)b (16.000 | 246.746)b (16.000 | 243.713)b (15.000 | 268.262)b (15.000 | 268.040)b (15.000 | 261.451)b (15.000 | 255.385)b (14.000 | 265.386)b (13.000 | 268.387)b (12.000 | 280.808)b (11.000 | 283.809)b (11.000 | 260.130)b (10.000 | 274.132)b (9.000 | 277.133)b (9.000 | 271.229)b (8.000 | 274.230)b (7.000 | 278.231)