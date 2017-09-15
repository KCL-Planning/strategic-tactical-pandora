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
Initial heuristic = 35.000
b (34.000 | 3.000)b (33.000 | 25.562)b (32.000 | 97.644)b (31.000 | 100.645)b (30.000 | 110.646)b (29.000 | 113.647)b (28.000 | 122.133)b (27.000 | 125.134)b (26.000 | 135.135)b (25.000 | 138.136)b (24.000 | 146.622)b (23.000 | 149.623)b (22.000 | 159.624)b (21.000 | 162.625)b (20.000 | 171.112)b (19.000 | 174.113)b (18.000 | 184.114)b (17.000 | 187.115)b (16.000 | 196.964)b (15.000 | 199.965)b (14.000 | 209.966)b (13.000 | 212.967)b (12.000 | 221.454)b (11.000 | 224.455)b (10.000 | 234.456)b (9.000 | 237.457)b (8.000 | 245.943)b (7.000 | 248.944)b (6.000 | 258.945)b (5.000 | 261.946)b (4.000 | 270.432)b (3.000 | 273.433)b (2.000 | 283.434)b (1.000 | 286.435)
; Plan found with metric 354.385
; States evaluated so far: 72
; Time 0.08
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 mission_site_start_point_1)  [22.561]
25.563: (correct_position auv0 mission_site_start_point_1)  [3.000]
28.564: (do_hover_fast auv0 mission_site_start_point_1 strategic_location_1)  [69.080]
97.645: (correct_position auv0 strategic_location_1)  [3.000]
100.646: (observe_inspection_point auv0 strategic_location_1 inspection_point_48)  [10.000]
110.647: (correct_position auv0 strategic_location_1)  [3.000]
113.648: (do_hover_fast auv0 strategic_location_1 strategic_location_2)  [8.485]
122.134: (correct_position auv0 strategic_location_2)  [3.000]
125.135: (observe_inspection_point auv0 strategic_location_2 inspection_point_49)  [10.000]
135.136: (correct_position auv0 strategic_location_2)  [3.000]
138.137: (do_hover_fast auv0 strategic_location_2 strategic_location_3)  [8.485]
146.623: (correct_position auv0 strategic_location_3)  [3.000]
149.624: (observe_inspection_point auv0 strategic_location_3 inspection_point_50)  [10.000]
159.625: (correct_position auv0 strategic_location_3)  [3.000]
162.626: (do_hover_fast auv0 strategic_location_3 strategic_location_4)  [8.485]
171.113: (correct_position auv0 strategic_location_4)  [3.000]
174.114: (observe_inspection_point auv0 strategic_location_4 inspection_point_51)  [10.000]
184.115: (correct_position auv0 strategic_location_4)  [3.000]
187.116: (do_hover_fast auv0 strategic_location_4 strategic_location_5)  [9.849]
196.965: (correct_position auv0 strategic_location_5)  [3.000]
199.966: (observe_inspection_point auv0 strategic_location_5 inspection_point_52)  [10.000]
209.967: (correct_position auv0 strategic_location_5)  [3.000]
212.968: (do_hover_fast auv0 strategic_location_5 strategic_location_6)  [8.485]
221.455: (correct_position auv0 strategic_location_6)  [3.000]
224.456: (observe_inspection_point auv0 strategic_location_6 inspection_point_53)  [10.000]
234.457: (correct_position auv0 strategic_location_6)  [3.000]
237.458: (do_hover_fast auv0 strategic_location_6 strategic_location_7)  [8.485]
245.944: (correct_position auv0 strategic_location_7)  [3.000]
248.945: (observe_inspection_point auv0 strategic_location_7 inspection_point_54)  [10.000]
258.946: (correct_position auv0 strategic_location_7)  [3.000]
261.947: (do_hover_fast auv0 strategic_location_7 strategic_location_8)  [8.485]
270.433: (correct_position auv0 strategic_location_8)  [3.000]
273.434: (observe_inspection_point auv0 strategic_location_8 inspection_point_55)  [10.000]
283.435: (correct_position auv0 strategic_location_8)  [3.000]
286.436: (do_hover_fast auv0 strategic_location_8 mission_site_start_point_1)  [67.948]

 * All goal deadlines now no later than 354.385

Resorting to best-first search
b (34.000 | 3.000)b (33.000 | 25.562)b (32.000 | 117.204)b (32.000 | 108.886)b (32.000 | 107.890)b (32.000 | 107.597)b (31.000 | 186.283)b (31.000 | 169.648)b (31.000 | 167.656)b (31.000 | 167.069)b (30.000 | 177.070)b (29.000 | 180.071)b (28.000 | 193.072)b (28.000 | 189.921)b (28.000 | 185.072)b (27.000 | 206.072)b (27.000 | 199.770)b (27.000 | 190.072)b (26.000 | 200.073)b (25.000 | 203.074)b (24.000 | 215.075)b (24.000 | 211.561)b (23.000 | 227.075)b (23.000 | 220.046)b (22.000 | 230.047)b (21.000 | 233.048)b (20.000 | 241.534)b (20.000 | 238.049)b (19.000 | 250.020)b (19.000 | 243.049)b (18.000 | 253.050)b (17.000 | 256.051)b (16.000 | 265.901)b (16.000 | 264.537)b (15.000 | 275.750)b (15.000 | 273.023)b (15.000 | 267.538)b (14.000 | 290.053)b (13.000 | 293.054)b (13.000 | 292.995)