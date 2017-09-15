Number of literals: 29
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_16) are preferable
Have identified that bigger values of (observed inspection_point_17) are preferable
Have identified that bigger values of (observed inspection_point_18) are preferable
Have identified that bigger values of (observed inspection_point_19) are preferable
Have identified that bigger values of (observed inspection_point_20) are preferable
Have identified that bigger values of (observed inspection_point_21) are preferable
Have identified that bigger values of (observed inspection_point_22) are preferable
Have identified that bigger values of (observed inspection_point_23) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 34.000
b (33.000 | 3.000)b (30.000 | 34.017)b (29.000 | 37.018)b (28.000 | 47.019)b (27.000 | 50.020)b (26.000 | 68.021)b (25.000 | 71.022)b (24.000 | 81.023)b (23.000 | 84.024)b (22.000 | 98.026)b (21.000 | 101.027)b (20.000 | 119.467)b (19.000 | 122.468)b (18.000 | 132.469)b (17.000 | 135.470)b (16.000 | 139.471)b (15.000 | 142.472)b (14.000 | 152.473)b (13.000 | 155.474)b (12.000 | 173.914)b (11.000 | 176.915)b (10.000 | 186.916)b (9.000 | 189.917)b (8.000 | 204.624)b (7.000 | 207.625)b (6.000 | 217.626)b (5.000 | 220.627)b (4.000 | 240.628)b (3.000 | 243.629)b (2.000 | 253.630)b (1.000 | 256.631)
; Plan found with metric 311.260
; States evaluated so far: 74
; Time 0.08
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [31.016]
34.018: (correct_position auv0 strategic_location_1)  [3.000]
37.019: (observe_inspection_point auv0 strategic_location_1 inspection_point_16)  [10.000]
47.020: (correct_position auv0 strategic_location_1)  [3.000]
50.021: (do_hover_fast auv0 strategic_location_1 strategic_location_5)  [18.000]
68.022: (correct_position auv0 strategic_location_5)  [3.000]
71.023: (observe_inspection_point auv0 strategic_location_5 inspection_point_20)  [10.000]
81.024: (correct_position auv0 strategic_location_5)  [3.000]
84.025: (do_hover_controlled auv0 strategic_location_5 strategic_location_4)  [4.000]
88.026: (observe_inspection_point auv0 strategic_location_4 inspection_point_19)  [10.000]
98.027: (correct_position auv0 strategic_location_4)  [3.000]
101.028: (do_hover_fast auv0 strategic_location_4 strategic_location_2)  [18.439]
119.468: (correct_position auv0 strategic_location_2)  [3.000]
122.469: (observe_inspection_point auv0 strategic_location_2 inspection_point_17)  [10.000]
132.470: (correct_position auv0 strategic_location_2)  [3.000]
135.471: (do_hover_fast auv0 strategic_location_2 strategic_location_3)  [4.000]
139.472: (correct_position auv0 strategic_location_3)  [3.000]
142.473: (observe_inspection_point auv0 strategic_location_3 inspection_point_18)  [10.000]
152.474: (correct_position auv0 strategic_location_3)  [3.000]
155.475: (do_hover_fast auv0 strategic_location_3 strategic_location_6)  [18.439]
173.915: (correct_position auv0 strategic_location_6)  [3.000]
176.916: (observe_inspection_point auv0 strategic_location_6 inspection_point_21)  [10.000]
186.917: (correct_position auv0 strategic_location_6)  [3.000]
189.918: (do_hover_fast auv0 strategic_location_6 strategic_location_7)  [14.705]
204.625: (correct_position auv0 strategic_location_7)  [3.000]
207.626: (observe_inspection_point auv0 strategic_location_7 inspection_point_22)  [10.000]
217.627: (correct_position auv0 strategic_location_7)  [3.000]
220.628: (do_hover_fast auv0 strategic_location_7 strategic_location_8)  [20.000]
240.629: (correct_position auv0 strategic_location_8)  [3.000]
243.630: (observe_inspection_point auv0 strategic_location_8 inspection_point_23)  [10.000]
253.631: (correct_position auv0 strategic_location_8)  [3.000]
256.632: (do_hover_fast auv0 strategic_location_8 mission_site_start_point_1)  [54.628]

 * All goal deadlines now no later than 311.260

Resorting to best-first search
b (33.000 | 3.000)b (30.000 | 34.017)b (29.000 | 65.033)b (28.000 | 75.034)b (27.000 | 78.035)b (26.000 | 96.036)b (25.000 | 114.036)b (24.000 | 124.037)b (23.000 | 127.038)b (22.000 | 141.040)b (21.000 | 144.041)b (20.000 | 162.481)b (20.000 | 162.042)b (20.000 | 148.042)b (19.000 | 180.264)b (19.000 | 180.042)b (19.000 | 152.042)b (18.000 | 162.043)b (17.000 | 165.044)b (16.000 | 183.045)b (16.000 | 179.751)b (16.000 | 176.718)b (15.000 | 201.267)b (15.000 | 201.045)b (15.000 | 194.456)b (15.000 | 188.390)b (14.000 | 198.391)b (13.000 | 201.392)b (12.000 | 213.813)b (11.000 | 228.980)b (11.000 | 226.233)b (10.000 | 236.234)b (9.000 | 239.235)b (8.000 | 243.236)b (7.000 | 247.236)b (6.000 | 257.237)b (5.000 | 260.238)