(define (problem pandora_mission_task)
(:domain pandora_domain_persistent)
(:objects
auv0 - vehicle
inspection_point_16 inspection_point_17 inspection_point_18 inspection_point_19 inspection_point_20 inspection_point_21 inspection_point_22 inspection_point_23 - inspectionpoint
mission_site_start_point_1 strategic_location_1 strategic_location_2 strategic_location_3 strategic_location_4 strategic_location_5 strategic_location_6 strategic_location_7 strategic_location_8 wp_AUV0 - Waypoint
)
(:init
(= (arm_calibration auv0) 0)
(not_illuminating auv0)
(near auv0 wp_auv0)

(waypoint_not_occupied mission_site_start_point_1)
(waypoint_not_occupied strategic_location_1)
(waypoint_not_occupied strategic_location_2)
(waypoint_not_occupied strategic_location_3)
(waypoint_not_occupied strategic_location_4)
(waypoint_not_occupied strategic_location_5)
(waypoint_not_occupied strategic_location_6)
(waypoint_not_occupied strategic_location_7)
(waypoint_not_occupied strategic_location_8)
(waypoint_not_occupied wp_AUV0)

(= (observed inspection_point_16) 0)
(= (observed inspection_point_17) 0)
(= (observed inspection_point_18) 0)
(= (observed inspection_point_19) 0)
(= (observed inspection_point_20) 0)
(= (observed inspection_point_21) 0)
(= (observed inspection_point_22) 0)
(= (observed inspection_point_23) 0)
(cansee auv0 strategic_location_1 inspection_point_16) (= (obs inspection_point_16 strategic_location_1) 1)
(cansee auv0 strategic_location_1 inspection_point_16) (= (obs inspection_point_16 strategic_location_1) 1)
(cansee auv0 strategic_location_2 inspection_point_17) (= (obs inspection_point_17 strategic_location_2) 1)
(cansee auv0 strategic_location_2 inspection_point_17) (= (obs inspection_point_17 strategic_location_2) 1)
(cansee auv0 strategic_location_3 inspection_point_18) (= (obs inspection_point_18 strategic_location_3) 1)
(cansee auv0 strategic_location_3 inspection_point_18) (= (obs inspection_point_18 strategic_location_3) 1)
(cansee auv0 strategic_location_4 inspection_point_19) (= (obs inspection_point_19 strategic_location_4) 1)
(cansee auv0 strategic_location_4 inspection_point_19) (= (obs inspection_point_19 strategic_location_4) 1)
(cansee auv0 strategic_location_5 inspection_point_20) (= (obs inspection_point_20 strategic_location_5) 1)
(cansee auv0 strategic_location_5 inspection_point_20) (= (obs inspection_point_20 strategic_location_5) 1)
(cansee auv0 strategic_location_6 inspection_point_21) (= (obs inspection_point_21 strategic_location_6) 1)
(cansee auv0 strategic_location_6 inspection_point_21) (= (obs inspection_point_21 strategic_location_6) 1)
(cansee auv0 strategic_location_7 inspection_point_22) (= (obs inspection_point_22 strategic_location_7) 1)
(cansee auv0 strategic_location_7 inspection_point_22) (= (obs inspection_point_22 strategic_location_7) 1)
(cansee auv0 strategic_location_8 inspection_point_23) (= (obs inspection_point_23 strategic_location_8) 1)
(cansee auv0 strategic_location_8 inspection_point_23) (= (obs inspection_point_23 strategic_location_8) 1)
(connected wp_AUV0 strategic_location_5)  (= (distance wp_AUV0 strategic_location_5) 45.1885)
(connected wp_AUV0 strategic_location_3)  (= (distance wp_AUV0 strategic_location_3) 29.7658)
(connected wp_AUV0 strategic_location_7)  (= (distance wp_AUV0 strategic_location_7) 33.9301)
(connected wp_AUV0 strategic_location_1)  (= (distance wp_AUV0 strategic_location_1) 31.0161)
(connected wp_AUV0 strategic_location_4)  (= (distance wp_AUV0 strategic_location_4) 44.3396)
(connected wp_AUV0 strategic_location_6)  (= (distance wp_AUV0 strategic_location_6) 46.1086)
(connected wp_AUV0 strategic_location_8)  (= (distance wp_AUV0 strategic_location_8) 44.173)
(connected wp_AUV0 mission_site_start_point_1)  (= (distance wp_AUV0 mission_site_start_point_1) 22.561)
(connected wp_AUV0 strategic_location_2)  (= (distance wp_AUV0 strategic_location_2) 32.3419)
(connected strategic_location_5 strategic_location_5)  (= (distance strategic_location_5 strategic_location_5) 0)
(connected strategic_location_5 strategic_location_5)  (= (distance strategic_location_5 strategic_location_5) 0)
(connected strategic_location_5 strategic_location_3)  (= (distance strategic_location_5 strategic_location_3) 18.1108)
(connected strategic_location_5 strategic_location_7)  (= (distance strategic_location_5 strategic_location_7) 13.1244)
(connected strategic_location_5 strategic_location_1)  (= (distance strategic_location_5 strategic_location_1) 18)
(connected strategic_location_5 strategic_location_4)  (= (distance strategic_location_5 strategic_location_4) 2)
(connected strategic_location_5 strategic_location_6)  (= (distance strategic_location_5 strategic_location_6) 2)
(connected strategic_location_5 strategic_location_8)  (= (distance strategic_location_5 strategic_location_8) 13.1244)
(connected strategic_location_5 mission_site_start_point_1)  (= (distance strategic_location_5 mission_site_start_point_1) 50.05)
(connected strategic_location_5 strategic_location_2)  (= (distance strategic_location_5 strategic_location_2) 18.1108)
(connected strategic_location_5 wp_AUV0)  (= (distance strategic_location_5 wp_AUV0) 45.1885)
(connected strategic_location_5 strategic_location_3)  (= (distance strategic_location_5 strategic_location_3) 18.1108)
(connected strategic_location_5 strategic_location_7)  (= (distance strategic_location_5 strategic_location_7) 13.1244)
(connected strategic_location_5 strategic_location_1)  (= (distance strategic_location_5 strategic_location_1) 18)
(connected strategic_location_5 strategic_location_4)  (= (distance strategic_location_5 strategic_location_4) 2)
(connected strategic_location_5 strategic_location_6)  (= (distance strategic_location_5 strategic_location_6) 2)
(connected strategic_location_5 strategic_location_8)  (= (distance strategic_location_5 strategic_location_8) 13.1244)
(connected strategic_location_5 mission_site_start_point_1)  (= (distance strategic_location_5 mission_site_start_point_1) 50.05)
(connected strategic_location_5 strategic_location_2)  (= (distance strategic_location_5 strategic_location_2) 18.1108)
(connected strategic_location_3 strategic_location_5)  (= (distance strategic_location_3 strategic_location_5) 18.1108)
(connected strategic_location_3 strategic_location_3)  (= (distance strategic_location_3 strategic_location_3) 0)
(connected strategic_location_3 strategic_location_3)  (= (distance strategic_location_3 strategic_location_3) 0)
(connected strategic_location_3 strategic_location_7)  (= (distance strategic_location_3 strategic_location_7) 12.4197)
(connected strategic_location_3 strategic_location_1)  (= (distance strategic_location_3 strategic_location_1) 2)
(connected strategic_location_3 strategic_location_4)  (= (distance strategic_location_3 strategic_location_4) 18)
(connected strategic_location_3 strategic_location_6)  (= (distance strategic_location_3 strategic_location_6) 18.4391)
(connected strategic_location_3 strategic_location_8)  (= (distance strategic_location_3 strategic_location_8) 15.3052)
(connected strategic_location_3 mission_site_start_point_1)  (= (distance strategic_location_3 mission_site_start_point_1) 40.3609)
(connected strategic_location_3 strategic_location_2)  (= (distance strategic_location_3 strategic_location_2) 4)
(connected strategic_location_3 wp_AUV0)  (= (distance strategic_location_3 wp_AUV0) 29.7658)
(connected strategic_location_3 strategic_location_7)  (= (distance strategic_location_3 strategic_location_7) 12.4197)
(connected strategic_location_3 strategic_location_1)  (= (distance strategic_location_3 strategic_location_1) 2)
(connected strategic_location_3 strategic_location_4)  (= (distance strategic_location_3 strategic_location_4) 18)
(connected strategic_location_3 strategic_location_6)  (= (distance strategic_location_3 strategic_location_6) 18.4391)
(connected strategic_location_3 strategic_location_8)  (= (distance strategic_location_3 strategic_location_8) 15.3052)
(connected strategic_location_3 mission_site_start_point_1)  (= (distance strategic_location_3 mission_site_start_point_1) 40.3609)
(connected strategic_location_3 strategic_location_2)  (= (distance strategic_location_3 strategic_location_2) 4)
(connected strategic_location_7 strategic_location_5)  (= (distance strategic_location_7 strategic_location_5) 13.1244)
(connected strategic_location_7 strategic_location_3)  (= (distance strategic_location_7 strategic_location_3) 12.4197)
(connected strategic_location_7 strategic_location_7)  (= (distance strategic_location_7 strategic_location_7) 0)
(connected strategic_location_7 strategic_location_7)  (= (distance strategic_location_7 strategic_location_7) 0)
(connected strategic_location_7 strategic_location_1)  (= (distance strategic_location_7 strategic_location_1) 13.7931)
(connected strategic_location_7 strategic_location_4)  (= (distance strategic_location_7 strategic_location_4) 11.6726)
(connected strategic_location_7 strategic_location_6)  (= (distance strategic_location_7 strategic_location_6) 14.7054)
(connected strategic_location_7 strategic_location_8)  (= (distance strategic_location_7 strategic_location_8) 20)
(connected strategic_location_7 mission_site_start_point_1)  (= (distance strategic_location_7 mission_site_start_point_1) 37.2055)
(connected strategic_location_7 strategic_location_2)  (= (distance strategic_location_7 strategic_location_2) 15.3052)
(connected strategic_location_7 wp_AUV0)  (= (distance strategic_location_7 wp_AUV0) 33.9301)
(connected strategic_location_7 strategic_location_1)  (= (distance strategic_location_7 strategic_location_1) 13.7931)
(connected strategic_location_7 strategic_location_4)  (= (distance strategic_location_7 strategic_location_4) 11.6726)
(connected strategic_location_7 strategic_location_6)  (= (distance strategic_location_7 strategic_location_6) 14.7054)
(connected strategic_location_7 strategic_location_8)  (= (distance strategic_location_7 strategic_location_8) 20)
(connected strategic_location_7 mission_site_start_point_1)  (= (distance strategic_location_7 mission_site_start_point_1) 37.2055)
(connected strategic_location_7 strategic_location_2)  (= (distance strategic_location_7 strategic_location_2) 15.3052)
(connected strategic_location_1 strategic_location_5)  (= (distance strategic_location_1 strategic_location_5) 18)
(connected strategic_location_1 strategic_location_3)  (= (distance strategic_location_1 strategic_location_3) 2)
(connected strategic_location_1 strategic_location_7)  (= (distance strategic_location_1 strategic_location_7) 13.7931)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
(connected strategic_location_1 strategic_location_4)  (= (distance strategic_location_1 strategic_location_4) 18.1108)
(connected strategic_location_1 strategic_location_6)  (= (distance strategic_location_1 strategic_location_6) 18.1108)
(connected strategic_location_1 strategic_location_8)  (= (distance strategic_location_1 strategic_location_8) 13.7931)
(connected strategic_location_1 mission_site_start_point_1)  (= (distance strategic_location_1 mission_site_start_point_1) 42.2493)
(connected strategic_location_1 strategic_location_2)  (= (distance strategic_location_1 strategic_location_2) 2)
(connected strategic_location_1 wp_AUV0)  (= (distance strategic_location_1 wp_AUV0) 31.0161)
(connected strategic_location_1 strategic_location_4)  (= (distance strategic_location_1 strategic_location_4) 18.1108)
(connected strategic_location_1 strategic_location_6)  (= (distance strategic_location_1 strategic_location_6) 18.1108)
(connected strategic_location_1 strategic_location_8)  (= (distance strategic_location_1 strategic_location_8) 13.7931)
(connected strategic_location_1 mission_site_start_point_1)  (= (distance strategic_location_1 mission_site_start_point_1) 42.2493)
(connected strategic_location_1 strategic_location_2)  (= (distance strategic_location_1 strategic_location_2) 2)
(connected strategic_location_4 strategic_location_5)  (= (distance strategic_location_4 strategic_location_5) 2)
(connected strategic_location_4 strategic_location_3)  (= (distance strategic_location_4 strategic_location_3) 18)
(connected strategic_location_4 strategic_location_7)  (= (distance strategic_location_4 strategic_location_7) 11.6726)
(connected strategic_location_4 strategic_location_1)  (= (distance strategic_location_4 strategic_location_1) 18.1108)
(connected strategic_location_4 strategic_location_4)  (= (distance strategic_location_4 strategic_location_4) 0)
(connected strategic_location_4 strategic_location_4)  (= (distance strategic_location_4 strategic_location_4) 0)
(connected strategic_location_4 strategic_location_6)  (= (distance strategic_location_4 strategic_location_6) 4)
(connected strategic_location_4 strategic_location_8)  (= (distance strategic_location_4 strategic_location_8) 14.7054)
(connected strategic_location_4 mission_site_start_point_1)  (= (distance strategic_location_4 mission_site_start_point_1) 48.4665)
(connected strategic_location_4 strategic_location_2)  (= (distance strategic_location_4 strategic_location_2) 18.4391)
(connected strategic_location_4 wp_AUV0)  (= (distance strategic_location_4 wp_AUV0) 44.3396)
(connected strategic_location_4 strategic_location_6)  (= (distance strategic_location_4 strategic_location_6) 4)
(connected strategic_location_4 strategic_location_8)  (= (distance strategic_location_4 strategic_location_8) 14.7054)
(connected strategic_location_4 mission_site_start_point_1)  (= (distance strategic_location_4 mission_site_start_point_1) 48.4665)
(connected strategic_location_4 strategic_location_2)  (= (distance strategic_location_4 strategic_location_2) 18.4391)
(connected strategic_location_6 strategic_location_5)  (= (distance strategic_location_6 strategic_location_5) 2)
(connected strategic_location_6 strategic_location_3)  (= (distance strategic_location_6 strategic_location_3) 18.4391)
(connected strategic_location_6 strategic_location_7)  (= (distance strategic_location_6 strategic_location_7) 14.7054)
(connected strategic_location_6 strategic_location_1)  (= (distance strategic_location_6 strategic_location_1) 18.1108)
(connected strategic_location_6 strategic_location_4)  (= (distance strategic_location_6 strategic_location_4) 4)
(connected strategic_location_6 strategic_location_6)  (= (distance strategic_location_6 strategic_location_6) 0)
(connected strategic_location_6 strategic_location_6)  (= (distance strategic_location_6 strategic_location_6) 0)
(connected strategic_location_6 strategic_location_8)  (= (distance strategic_location_6 strategic_location_8) 11.6726)
(connected strategic_location_6 mission_site_start_point_1)  (= (distance strategic_location_6 mission_site_start_point_1) 51.6624)
(connected strategic_location_6 strategic_location_2)  (= (distance strategic_location_6 strategic_location_2) 18)
(connected strategic_location_6 wp_AUV0)  (= (distance strategic_location_6 wp_AUV0) 46.1086)
(connected strategic_location_6 strategic_location_8)  (= (distance strategic_location_6 strategic_location_8) 11.6726)
(connected strategic_location_6 mission_site_start_point_1)  (= (distance strategic_location_6 mission_site_start_point_1) 51.6624)
(connected strategic_location_6 strategic_location_2)  (= (distance strategic_location_6 strategic_location_2) 18)
(connected strategic_location_8 strategic_location_5)  (= (distance strategic_location_8 strategic_location_5) 13.1244)
(connected strategic_location_8 strategic_location_3)  (= (distance strategic_location_8 strategic_location_3) 15.3052)
(connected strategic_location_8 strategic_location_7)  (= (distance strategic_location_8 strategic_location_7) 20)
(connected strategic_location_8 strategic_location_1)  (= (distance strategic_location_8 strategic_location_1) 13.7931)
(connected strategic_location_8 strategic_location_4)  (= (distance strategic_location_8 strategic_location_4) 14.7054)
(connected strategic_location_8 strategic_location_6)  (= (distance strategic_location_8 strategic_location_6) 11.6726)
(connected strategic_location_8 strategic_location_8)  (= (distance strategic_location_8 strategic_location_8) 0)
(connected strategic_location_8 strategic_location_8)  (= (distance strategic_location_8 strategic_location_8) 0)
(connected strategic_location_8 mission_site_start_point_1)  (= (distance strategic_location_8 mission_site_start_point_1) 54.6283)
(connected strategic_location_8 strategic_location_2)  (= (distance strategic_location_8 strategic_location_2) 12.4197)
(connected strategic_location_8 wp_AUV0)  (= (distance strategic_location_8 wp_AUV0) 44.173)
(connected strategic_location_8 mission_site_start_point_1)  (= (distance strategic_location_8 mission_site_start_point_1) 54.6283)
(connected strategic_location_8 strategic_location_2)  (= (distance strategic_location_8 strategic_location_2) 12.4197)
(connected mission_site_start_point_1 strategic_location_5)  (= (distance mission_site_start_point_1 strategic_location_5) 50.05)
(connected mission_site_start_point_1 strategic_location_3)  (= (distance mission_site_start_point_1 strategic_location_3) 40.3609)
(connected mission_site_start_point_1 strategic_location_7)  (= (distance mission_site_start_point_1 strategic_location_7) 37.2055)
(connected mission_site_start_point_1 strategic_location_1)  (= (distance mission_site_start_point_1 strategic_location_1) 42.2493)
(connected mission_site_start_point_1 strategic_location_4)  (= (distance mission_site_start_point_1 strategic_location_4) 48.4665)
(connected mission_site_start_point_1 strategic_location_6)  (= (distance mission_site_start_point_1 strategic_location_6) 51.6624)
(connected mission_site_start_point_1 strategic_location_8)  (= (distance mission_site_start_point_1 strategic_location_8) 54.6283)
(connected mission_site_start_point_1 mission_site_start_point_1)  (= (distance mission_site_start_point_1 mission_site_start_point_1) 0)
(connected mission_site_start_point_1 mission_site_start_point_1)  (= (distance mission_site_start_point_1 mission_site_start_point_1) 0)
(connected mission_site_start_point_1 strategic_location_2)  (= (distance mission_site_start_point_1 strategic_location_2) 44.1475)
(connected mission_site_start_point_1 wp_AUV0)  (= (distance mission_site_start_point_1 wp_AUV0) 22.561)
(connected mission_site_start_point_1 strategic_location_2)  (= (distance mission_site_start_point_1 strategic_location_2) 44.1475)
(connected strategic_location_2 strategic_location_5)  (= (distance strategic_location_2 strategic_location_5) 18.1108)
(connected strategic_location_2 strategic_location_3)  (= (distance strategic_location_2 strategic_location_3) 4)
(connected strategic_location_2 strategic_location_7)  (= (distance strategic_location_2 strategic_location_7) 15.3052)
(connected strategic_location_2 strategic_location_1)  (= (distance strategic_location_2 strategic_location_1) 2)
(connected strategic_location_2 strategic_location_4)  (= (distance strategic_location_2 strategic_location_4) 18.4391)
(connected strategic_location_2 strategic_location_6)  (= (distance strategic_location_2 strategic_location_6) 18)
(connected strategic_location_2 strategic_location_8)  (= (distance strategic_location_2 strategic_location_8) 12.4197)
(connected strategic_location_2 mission_site_start_point_1)  (= (distance strategic_location_2 mission_site_start_point_1) 44.1475)
(connected strategic_location_2 strategic_location_2)  (= (distance strategic_location_2 strategic_location_2) 0)
(connected strategic_location_2 strategic_location_2)  (= (distance strategic_location_2 strategic_location_2) 0)
(connected strategic_location_2 wp_AUV0)  (= (distance strategic_location_2 wp_AUV0) 32.3419)
)
(:goal (and
(>= (observed inspection_point_16) 1)
(>= (observed inspection_point_17) 1)
(>= (observed inspection_point_18) 1)
(>= (observed inspection_point_19) 1)
(>= (observed inspection_point_20) 1)
(>= (observed inspection_point_21) 1)
(>= (observed inspection_point_22) 1)
(>= (observed inspection_point_23) 1)
(near auv0 mission_site_start_point_1)
)))