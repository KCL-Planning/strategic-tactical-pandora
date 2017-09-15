Number of literals: 29
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_56) are preferable
Have identified that bigger values of (observed inspection_point_57) are preferable
Have identified that bigger values of (observed inspection_point_58) are preferable
Have identified that bigger values of (observed inspection_point_59) are preferable
Have identified that bigger values of (observed inspection_point_60) are preferable
Have identified that bigger values of (observed inspection_point_61) are preferable
Have identified that bigger values of (observed inspection_point_62) are preferable
Have identified that bigger values of (observed inspection_point_63) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 34.000
b (33.000 | 3.000)b (32.000 | 45.721)b (31.000 | 48.722)b (30.000 | 58.723)b (29.000 | 61.724)b (28.000 | 70.210)b (27.000 | 73.211)b (26.000 | 83.212)b (25.000 | 86.213)b (24.000 | 94.700)b (23.000 | 97.701)b (22.000 | 107.702)b (21.000 | 110.703)b (20.000 | 119.189)b (19.000 | 122.190)b (18.000 | 132.191)b (17.000 | 135.192)b (16.000 | 145.042)b (15.000 | 148.043)b (14.000 | 158.044)b (13.000 | 161.045)b (12.000 | 169.531)b (11.000 | 172.532)b (10.000 | 182.533)b (9.000 | 185.534)b (8.000 | 194.020)b (7.000 | 197.021)b (6.000 | 207.022)b (5.000 | 210.023)b (4.000 | 218.510)b (3.000 | 221.511)b (2.000 | 231.512)b (1.000 | 234.513)
; Plan found with metric 258.617
; States evaluated so far: 71
; Time 0.06
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [42.720]
45.722: (correct_position auv0 strategic_location_1)  [3.000]
48.723: (observe_inspection_point auv0 strategic_location_1 inspection_point_56)  [10.000]
58.724: (correct_position auv0 strategic_location_1)  [3.000]
61.725: (do_hover_fast auv0 strategic_location_1 strategic_location_2)  [8.485]
70.211: (correct_position auv0 strategic_location_2)  [3.000]
73.212: (observe_inspection_point auv0 strategic_location_2 inspection_point_57)  [10.000]
83.213: (correct_position auv0 strategic_location_2)  [3.000]
86.214: (do_hover_fast auv0 strategic_location_2 strategic_location_3)  [8.485]
94.701: (correct_position auv0 strategic_location_3)  [3.000]
97.702: (observe_inspection_point auv0 strategic_location_3 inspection_point_58)  [10.000]
107.703: (correct_position auv0 strategic_location_3)  [3.000]
110.704: (do_hover_fast auv0 strategic_location_3 strategic_location_4)  [8.485]
119.190: (correct_position auv0 strategic_location_4)  [3.000]
122.191: (observe_inspection_point auv0 strategic_location_4 inspection_point_59)  [10.000]
132.192: (correct_position auv0 strategic_location_4)  [3.000]
135.193: (do_hover_fast auv0 strategic_location_4 strategic_location_5)  [9.849]
145.043: (correct_position auv0 strategic_location_5)  [3.000]
148.044: (observe_inspection_point auv0 strategic_location_5 inspection_point_60)  [10.000]
158.045: (correct_position auv0 strategic_location_5)  [3.000]
161.046: (do_hover_fast auv0 strategic_location_5 strategic_location_6)  [8.485]
169.532: (correct_position auv0 strategic_location_6)  [3.000]
172.533: (observe_inspection_point auv0 strategic_location_6 inspection_point_61)  [10.000]
182.534: (correct_position auv0 strategic_location_6)  [3.000]
185.535: (do_hover_fast auv0 strategic_location_6 strategic_location_7)  [8.485]
194.021: (correct_position auv0 strategic_location_7)  [3.000]
197.022: (observe_inspection_point auv0 strategic_location_7 inspection_point_62)  [10.000]
207.023: (correct_position auv0 strategic_location_7)  [3.000]
210.024: (do_hover_fast auv0 strategic_location_7 strategic_location_8)  [8.485]
218.511: (correct_position auv0 strategic_location_8)  [3.000]
221.512: (observe_inspection_point auv0 strategic_location_8 inspection_point_63)  [10.000]
231.513: (correct_position auv0 strategic_location_8)  [3.000]
234.514: (do_hover_fast auv0 strategic_location_8 mission_site_start_point_1)  [24.104]

 * All goal deadlines now no later than 258.617

Resorting to best-first search
b (33.000 | 3.000)b (32.000 | 45.721)b (32.000 | 42.511)b (32.000 | 41.679)b (31.000 | 88.441)b (31.000 | 82.020)b (31.000 | 80.357)b (30.000 | 90.358)b (29.000 | 93.359)b (28.000 | 103.209)b (28.000 | 98.360)b (27.000 | 113.058)b (27.000 | 103.360)b (26.000 | 113.361)b (25.000 | 116.362)b (24.000 | 124.849)b (23.000 | 133.334)b (22.000 | 143.335)b (21.000 | 146.336)b (20.000 | 154.822)b (20.000 | 151.337)b (19.000 | 163.308)b (19.000 | 156.337)b (18.000 | 166.338)b (17.000 | 169.339)b (16.000 | 179.189)b (16.000 | 177.825)b (15.000 | 189.038)b (15.000 | 186.311)b (14.000 | 196.312)b (13.000 | 199.313)b (12.000 | 204.314)b (11.000 | 209.314)b (11.000 | 207.315)