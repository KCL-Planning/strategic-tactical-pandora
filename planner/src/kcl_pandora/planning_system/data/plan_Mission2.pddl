Number of literals: 29
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_10) are preferable
Have identified that bigger values of (observed inspection_point_11) are preferable
Have identified that bigger values of (observed inspection_point_12) are preferable
Have identified that bigger values of (observed inspection_point_13) are preferable
Have identified that bigger values of (observed inspection_point_14) are preferable
Have identified that bigger values of (observed inspection_point_15) are preferable
Have identified that bigger values of (observed inspection_point_8) are preferable
Have identified that bigger values of (observed inspection_point_9) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 34.000
b (33.000 | 3.000)b (32.000 | 25.562)b (31.000 | 77.031)b (30.000 | 80.032)b (29.000 | 84.033)b (28.000 | 94.034)b (27.000 | 97.035)b (26.000 | 115.036)b (25.000 | 118.037)b (24.000 | 128.038)b (23.000 | 131.039)b (22.000 | 145.041)b (21.000 | 148.042)b (20.000 | 166.482)b (19.000 | 169.483)b (18.000 | 179.484)b (17.000 | 182.485)b (16.000 | 186.486)b (15.000 | 189.487)b (14.000 | 199.488)b (13.000 | 202.489)b (12.000 | 227.911)b (11.000 | 230.912)b (10.000 | 243.333)b (9.000 | 246.334)b (8.000 | 274.642)b (7.000 | 277.643)b (6.000 | 292.949)b (5.000 | 295.950)b (4.000 | 327.393)b (3.000 | 330.394)b (2.000 | 348.834)b (1.000 | 351.835)
; Plan found with metric 400.302
; States evaluated so far: 84
; Time 0.10
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 mission_site_start_point_1)  [22.561]
25.563: (correct_position auv0 mission_site_start_point_1)  [3.000]
28.564: (do_hover_fast auv0 mission_site_start_point_1 strategic_location_3)  [48.466]
77.032: (correct_position auv0 strategic_location_3)  [3.000]
80.033: (do_hover_controlled auv0 strategic_location_3 strategic_location_1)  [4.000]
84.034: (observe_inspection_point auv0 strategic_location_1 inspection_point_8)  [10.000]
94.035: (correct_position auv0 strategic_location_1)  [3.000]
97.036: (do_hover_fast auv0 strategic_location_1 strategic_location_5)  [18.000]
115.037: (correct_position auv0 strategic_location_5)  [3.000]
118.038: (observe_inspection_point auv0 strategic_location_5 inspection_point_12)  [10.000]
128.039: (correct_position auv0 strategic_location_5)  [3.000]
131.040: (do_hover_controlled auv0 strategic_location_5 strategic_location_4)  [4.000]
135.041: (observe_inspection_point auv0 strategic_location_4 inspection_point_11)  [10.000]
145.042: (correct_position auv0 strategic_location_4)  [3.000]
148.043: (do_hover_fast auv0 strategic_location_4 strategic_location_2)  [18.439]
166.483: (correct_position auv0 strategic_location_2)  [3.000]
169.484: (observe_inspection_point auv0 strategic_location_2 inspection_point_9)  [10.000]
179.485: (correct_position auv0 strategic_location_2)  [3.000]
182.486: (do_hover_fast auv0 strategic_location_2 strategic_location_3)  [4.000]
186.487: (correct_position auv0 strategic_location_3)  [3.000]
189.488: (observe_inspection_point auv0 strategic_location_3 inspection_point_10)  [10.000]
199.489: (correct_position auv0 strategic_location_3)  [3.000]
202.490: (do_hover_fast auv0 strategic_location_3 strategic_location_7)  [12.420]
214.910: (correct_position auv0 strategic_location_7)  [3.000]
217.911: (observe_inspection_point auv0 strategic_location_7 inspection_point_14)  [10.000]
227.912: (correct_position auv0 strategic_location_7)  [3.000]
230.913: (do_hover_fast auv0 strategic_location_7 strategic_location_3)  [12.420]
243.334: (correct_position auv0 strategic_location_3)  [3.000]
246.335: (do_hover_fast auv0 strategic_location_3 strategic_location_8)  [15.305]
261.641: (correct_position auv0 strategic_location_8)  [3.000]
264.642: (observe_inspection_point auv0 strategic_location_8 inspection_point_15)  [10.000]
274.643: (correct_position auv0 strategic_location_8)  [3.000]
277.644: (do_hover_fast auv0 strategic_location_8 strategic_location_3)  [15.305]
292.950: (correct_position auv0 strategic_location_3)  [3.000]
295.951: (do_hover_fast auv0 strategic_location_3 strategic_location_6)  [18.439]
314.392: (correct_position auv0 strategic_location_6)  [3.000]
317.393: (observe_inspection_point auv0 strategic_location_6 inspection_point_13)  [10.000]
327.394: (correct_position auv0 strategic_location_6)  [3.000]
330.394: (do_hover_fast auv0 strategic_location_6 strategic_location_3)  [18.439]
348.835: (correct_position auv0 strategic_location_3)  [3.000]
351.836: (do_hover_fast auv0 strategic_location_3 mission_site_start_point_1)  [48.466]

 * All goal deadlines now no later than 400.302

Resorting to best-first search
b (33.000 | 3.000)b (32.000 | 25.562)b (31.000 | 96.590)b (30.000 | 145.057)b (29.000 | 149.058)b (28.000 | 159.059)b (27.000 | 162.060)b (26.000 | 180.061)b (25.000 | 198.061)b (24.000 | 208.062)b (23.000 | 211.063)b (22.000 | 225.065)b (21.000 | 228.066)b (20.000 | 246.506)b (20.000 | 246.067)b (20.000 | 232.067)b (19.000 | 264.289)b (19.000 | 264.067)b (19.000 | 236.067)b (18.000 | 246.068)b (17.000 | 249.069)b (16.000 | 267.070)b (16.000 | 263.775)b (16.000 | 260.743)b (15.000 | 285.292)b (15.000 | 285.070)b (15.000 | 278.481)b (15.000 | 272.415)b (14.000 | 282.416)b (13.000 | 285.417)b (12.000 | 297.838)b (11.000 | 313.004)b (11.000 | 310.258)b (10.000 | 320.259)b (9.000 | 323.260)b (8.000 | 327.261)b (7.000 | 331.261)b (6.000 | 341.262)b (5.000 | 344.263)