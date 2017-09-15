Number of literals: 29
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_48) are preferable
Have identified that bigger values of (observed inspection_point_49) are preferable
Have identified that bigger values of (observed inspection_point_50) are preferable
Have identified that bigger values of (observed inspection_point_51) are preferable
Have identified that bigger values of (observed inspection_point_52) are preferable
Have identified that bigger values of (observed inspection_point_53) are preferable
Have identified that bigger values of (observed inspection_point_54) are preferable
Have identified that bigger values of (observed inspection_point_55) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 34.000
b (33.000 | 3.000)b (32.000 | 50.719)b (31.000 | 53.720)b (30.000 | 63.721)b (29.000 | 66.722)b (28.000 | 75.208)b (27.000 | 78.209)b (26.000 | 88.210)b (25.000 | 91.211)b (24.000 | 99.697)b (23.000 | 102.698)b (22.000 | 112.699)b (21.000 | 115.700)b (20.000 | 124.187)b (19.000 | 127.188)b (18.000 | 137.189)b (17.000 | 140.190)b (16.000 | 150.040)b (15.000 | 153.041)b (14.000 | 163.042)b (13.000 | 166.043)b (12.000 | 174.529)b (11.000 | 177.530)b (10.000 | 187.531)b (9.000 | 190.532)b (8.000 | 199.018)b (7.000 | 202.019)b (6.000 | 212.020)b (5.000 | 215.021)b (4.000 | 223.507)b (3.000 | 226.508)b (2.000 | 236.509)b (1.000 | 239.510)
; Plan found with metric 307.460
; States evaluated so far: 71
; Time 0.06
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [47.718]
50.720: (correct_position auv0 strategic_location_1)  [3.000]
53.721: (observe_inspection_point auv0 strategic_location_1 inspection_point_48)  [10.000]
63.722: (correct_position auv0 strategic_location_1)  [3.000]
66.723: (do_hover_fast auv0 strategic_location_1 strategic_location_2)  [8.485]
75.209: (correct_position auv0 strategic_location_2)  [3.000]
78.210: (observe_inspection_point auv0 strategic_location_2 inspection_point_49)  [10.000]
88.211: (correct_position auv0 strategic_location_2)  [3.000]
91.212: (do_hover_fast auv0 strategic_location_2 strategic_location_3)  [8.485]
99.698: (correct_position auv0 strategic_location_3)  [3.000]
102.699: (observe_inspection_point auv0 strategic_location_3 inspection_point_50)  [10.000]
112.700: (correct_position auv0 strategic_location_3)  [3.000]
115.701: (do_hover_fast auv0 strategic_location_3 strategic_location_4)  [8.485]
124.188: (correct_position auv0 strategic_location_4)  [3.000]
127.189: (observe_inspection_point auv0 strategic_location_4 inspection_point_51)  [10.000]
137.190: (correct_position auv0 strategic_location_4)  [3.000]
140.191: (do_hover_fast auv0 strategic_location_4 strategic_location_5)  [9.849]
150.041: (correct_position auv0 strategic_location_5)  [3.000]
153.042: (observe_inspection_point auv0 strategic_location_5 inspection_point_52)  [10.000]
163.043: (correct_position auv0 strategic_location_5)  [3.000]
166.044: (do_hover_fast auv0 strategic_location_5 strategic_location_6)  [8.485]
174.530: (correct_position auv0 strategic_location_6)  [3.000]
177.531: (observe_inspection_point auv0 strategic_location_6 inspection_point_53)  [10.000]
187.532: (correct_position auv0 strategic_location_6)  [3.000]
190.533: (do_hover_fast auv0 strategic_location_6 strategic_location_7)  [8.485]
199.019: (correct_position auv0 strategic_location_7)  [3.000]
202.020: (observe_inspection_point auv0 strategic_location_7 inspection_point_54)  [10.000]
212.021: (correct_position auv0 strategic_location_7)  [3.000]
215.022: (do_hover_fast auv0 strategic_location_7 strategic_location_8)  [8.485]
223.508: (correct_position auv0 strategic_location_8)  [3.000]
226.509: (observe_inspection_point auv0 strategic_location_8 inspection_point_55)  [10.000]
236.510: (correct_position auv0 strategic_location_8)  [3.000]
239.511: (do_hover_fast auv0 strategic_location_8 mission_site_start_point_1)  [67.948]

 * All goal deadlines now no later than 307.460

Resorting to best-first search
b (33.000 | 3.000)b (32.000 | 50.719)b (32.000 | 42.460)b (32.000 | 41.627)b (31.000 | 98.437)b (31.000 | 81.919)b (31.000 | 80.254)b (30.000 | 90.255)b (29.000 | 93.256)b (28.000 | 106.257)b (28.000 | 103.106)b (28.000 | 98.257)b (27.000 | 119.257)b (27.000 | 112.955)b (27.000 | 103.257)b (26.000 | 113.258)b (25.000 | 116.259)b (24.000 | 128.260)b (24.000 | 124.745)b (23.000 | 140.260)b (23.000 | 133.230)b (22.000 | 143.231)b (21.000 | 146.232)b (20.000 | 154.719)b (20.000 | 151.233)b (19.000 | 163.204)b (19.000 | 156.233)b (18.000 | 166.234)b (17.000 | 169.235)b (16.000 | 179.085)b (16.000 | 177.722)b (15.000 | 188.934)b (15.000 | 186.207)b (14.000 | 196.208)b (13.000 | 199.209)b (12.000 | 204.210)b (11.000 | 209.210)b (10.000 | 219.211)b (9.000 | 222.212)b (8.000 | 230.698)b (7.000 | 239.183)b (7.000 | 233.699)