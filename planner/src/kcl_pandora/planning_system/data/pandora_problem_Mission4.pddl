(define (problem pandora_mission_task)
(:domain pandora_domain_persistent)
(:objects
auv0 - vehicle
inspection_point_24 inspection_point_25 inspection_point_26 inspection_point_27 inspection_point_28 inspection_point_29 inspection_point_30 inspection_point_31 - inspectionpoint
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

(= (observed inspection_point_24) 0)
(= (observed inspection_point_25) 0)
(= (observed inspection_point_26) 0)
(= (observed inspection_point_27) 0)
(= (observed inspection_point_28) 0)
(= (observed inspection_point_29) 0)
(= (observed inspection_point_30) 0)
(= (observed inspection_point_31) 0)
(cansee auv0 strategic_location_1 inspection_point_24) (= (obs inspection_point_24 strategic_location_1) 1)
(cansee auv0 strategic_location_1 inspection_point_24) (= (obs inspection_point_24 strategic_location_1) 1)
(cansee auv0 strategic_location_2 inspection_point_25) (= (obs inspection_point_25 strategic_location_2) 1)
(cansee auv0 strategic_location_2 inspection_point_25) (= (obs inspection_point_25 strategic_location_2) 1)
(cansee auv0 strategic_location_3 inspection_point_26) (= (obs inspection_point_26 strategic_location_3) 1)
(cansee auv0 strategic_location_3 inspection_point_26) (= (obs inspection_point_26 strategic_location_3) 1)
(cansee auv0 strategic_location_4 inspection_point_27) (= (obs inspection_point_27 strategic_location_4) 1)
(cansee auv0 strategic_location_4 inspection_point_27) (= (obs inspection_point_27 strategic_location_4) 1)
(cansee auv0 strategic_location_5 inspection_point_28) (= (obs inspection_point_28 strategic_location_5) 1)
(cansee auv0 strategic_location_5 inspection_point_28) (= (obs inspection_point_28 strategic_location_5) 1)
(cansee auv0 strategic_location_6 inspection_point_29) (= (obs inspection_point_29 strategic_location_6) 1)
(cansee auv0 strategic_location_6 inspection_point_29) (= (obs inspection_point_29 strategic_location_6) 1)
(cansee auv0 strategic_location_7 inspection_point_30) (= (obs inspection_point_30 strategic_location_7) 1)
(cansee auv0 strategic_location_7 inspection_point_30) (= (obs inspection_point_30 strategic_location_7) 1)
(cansee auv0 strategic_location_8 inspection_point_31) (= (obs inspection_point_31 strategic_location_8) 1)
(cansee auv0 strategic_location_8 inspection_point_31) (= (obs inspection_point_31 strategic_location_8) 1)
(connected wp_AUV0 strategic_location_6)  (= (distance wp_AUV0 strategic_location_6) 70.7531)
(connected wp_AUV0 strategic_location_1)  (= (distance wp_AUV0 strategic_location_1) 64.5136)
(connected wp_AUV0 strategic_location_3)  (= (distance wp_AUV0 strategic_location_3) 66.3777)
(connected wp_AUV0 strategic_location_4)  (= (distance wp_AUV0 strategic_location_4) 74.0675)
(connected wp_AUV0 strategic_location_8)  (= (distance wp_AUV0 strategic_location_8) 59.5924)
(connected wp_AUV0 strategic_location_2)  (= (distance wp_AUV0 strategic_location_2) 62.6578)
(connected wp_AUV0 strategic_location_7)  (= (distance wp_AUV0 strategic_location_7) 77.1443)
(connected wp_AUV0 mission_site_start_point_1)  (= (distance wp_AUV0 mission_site_start_point_1) 22.561)
(connected wp_AUV0 strategic_location_5)  (= (distance wp_AUV0 strategic_location_5) 72.4017)
(connected strategic_location_6 strategic_location_6)  (= (distance strategic_location_6 strategic_location_6) 0)
(connected strategic_location_6 strategic_location_6)  (= (distance strategic_location_6 strategic_location_6) 0)
(connected strategic_location_6 strategic_location_1)  (= (distance strategic_location_6 strategic_location_1) 18.1108)
(connected strategic_location_6 strategic_location_3)  (= (distance strategic_location_6 strategic_location_3) 18.4391)
(connected strategic_location_6 strategic_location_4)  (= (distance strategic_location_6 strategic_location_4) 4)
(connected strategic_location_6 strategic_location_8)  (= (distance strategic_location_6 strategic_location_8) 11.6726)
(connected strategic_location_6 strategic_location_2)  (= (distance strategic_location_6 strategic_location_2) 18)
(connected strategic_location_6 strategic_location_7)  (= (distance strategic_location_6 strategic_location_7) 14.7054)
(connected strategic_location_6 mission_site_start_point_1)  (= (distance strategic_location_6 mission_site_start_point_1) 48.4665)
(connected strategic_location_6 strategic_location_5)  (= (distance strategic_location_6 strategic_location_5) 2)
(connected strategic_location_6 wp_AUV0)  (= (distance strategic_location_6 wp_AUV0) 70.7531)
(connected strategic_location_6 strategic_location_1)  (= (distance strategic_location_6 strategic_location_1) 18.1108)
(connected strategic_location_6 strategic_location_3)  (= (distance strategic_location_6 strategic_location_3) 18.4391)
(connected strategic_location_6 strategic_location_4)  (= (distance strategic_location_6 strategic_location_4) 4)
(connected strategic_location_6 strategic_location_8)  (= (distance strategic_location_6 strategic_location_8) 11.6726)
(connected strategic_location_6 strategic_location_2)  (= (distance strategic_location_6 strategic_location_2) 18)
(connected strategic_location_6 strategic_location_7)  (= (distance strategic_location_6 strategic_location_7) 14.7054)
(connected strategic_location_6 mission_site_start_point_1)  (= (distance strategic_location_6 mission_site_start_point_1) 48.4665)
(connected strategic_location_6 strategic_location_5)  (= (distance strategic_location_6 strategic_location_5) 2)
(connected strategic_location_1 strategic_location_6)  (= (distance strategic_location_1 strategic_location_6) 18.1108)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
(connected strategic_location_1 strategic_location_3)  (= (distance strategic_location_1 strategic_location_3) 2)
(connected strategic_location_1 strategic_location_4)  (= (distance strategic_location_1 strategic_location_4) 18.1108)
(connected strategic_location_1 strategic_location_8)  (= (distance strategic_location_1 strategic_location_8) 13.7931)
(connected strategic_location_1 strategic_location_2)  (= (distance strategic_location_1 strategic_location_2) 2)
(connected strategic_location_1 strategic_location_7)  (= (distance strategic_location_1 strategic_location_7) 13.7931)
(connected strategic_location_1 mission_site_start_point_1)  (= (distance strategic_location_1 mission_site_start_point_1) 42.2493)
(connected strategic_location_1 strategic_location_5)  (= (distance strategic_location_1 strategic_location_5) 18)
(connected strategic_location_1 wp_AUV0)  (= (distance strategic_location_1 wp_AUV0) 64.5136)
(connected strategic_location_1 strategic_location_3)  (= (distance strategic_location_1 strategic_location_3) 2)
(connected strategic_location_1 strategic_location_4)  (= (distance strategic_location_1 strategic_location_4) 18.1108)
(connected strategic_location_1 strategic_location_8)  (= (distance strategic_location_1 strategic_location_8) 13.7931)
(connected strategic_location_1 strategic_location_2)  (= (distance strategic_location_1 strategic_location_2) 2)
(connected strategic_location_1 strategic_location_7)  (= (distance strategic_location_1 strategic_location_7) 13.7931)
(connected strategic_location_1 mission_site_start_point_1)  (= (distance strategic_location_1 mission_site_start_point_1) 42.2493)
(connected strategic_location_1 strategic_location_5)  (= (distance strategic_location_1 strategic_location_5) 18)
(connected strategic_location_3 strategic_location_6)  (= (distance strategic_location_3 strategic_location_6) 18.4391)
(connected strategic_location_3 strategic_location_1)  (= (distance strategic_location_3 strategic_location_1) 2)
(connected strategic_location_3 strategic_location_3)  (= (distance strategic_location_3 strategic_location_3) 0)
(connected strategic_location_3 strategic_location_3)  (= (distance strategic_location_3 strategic_location_3) 0)
(connected strategic_location_3 strategic_location_4)  (= (distance strategic_location_3 strategic_location_4) 18)
(connected strategic_location_3 strategic_location_8)  (= (distance strategic_location_3 strategic_location_8) 15.3052)
(connected strategic_location_3 strategic_location_2)  (= (distance strategic_location_3 strategic_location_2) 4)
(connected strategic_location_3 strategic_location_7)  (= (distance strategic_location_3 strategic_location_7) 12.4197)
(connected strategic_location_3 mission_site_start_point_1)  (= (distance strategic_location_3 mission_site_start_point_1) 44.1475)
(connected strategic_location_3 strategic_location_5)  (= (distance strategic_location_3 strategic_location_5) 18.1108)
(connected strategic_location_3 wp_AUV0)  (= (distance strategic_location_3 wp_AUV0) 66.3777)
(connected strategic_location_3 strategic_location_4)  (= (distance strategic_location_3 strategic_location_4) 18)
(connected strategic_location_3 strategic_location_8)  (= (distance strategic_location_3 strategic_location_8) 15.3052)
(connected strategic_location_3 strategic_location_2)  (= (distance strategic_location_3 strategic_location_2) 4)
(connected strategic_location_3 strategic_location_7)  (= (distance strategic_location_3 strategic_location_7) 12.4197)
(connected strategic_location_3 mission_site_start_point_1)  (= (distance strategic_location_3 mission_site_start_point_1) 44.1475)
(connected strategic_location_3 strategic_location_5)  (= (distance strategic_location_3 strategic_location_5) 18.1108)
(connected strategic_location_4 strategic_location_6)  (= (distance strategic_location_4 strategic_location_6) 4)
(connected strategic_location_4 strategic_location_1)  (= (distance strategic_location_4 strategic_location_1) 18.1108)
(connected strategic_location_4 strategic_location_3)  (= (distance strategic_location_4 strategic_location_3) 18)
(connected strategic_location_4 strategic_location_4)  (= (distance strategic_location_4 strategic_location_4) 0)
(connected strategic_location_4 strategic_location_4)  (= (distance strategic_location_4 strategic_location_4) 0)
(connected strategic_location_4 strategic_location_8)  (= (distance strategic_location_4 strategic_location_8) 14.7054)
(connected strategic_location_4 strategic_location_2)  (= (distance strategic_location_4 strategic_location_2) 18.4391)
(connected strategic_location_4 strategic_location_7)  (= (distance strategic_location_4 strategic_location_7) 11.6726)
(connected strategic_location_4 mission_site_start_point_1)  (= (distance strategic_location_4 mission_site_start_point_1) 51.6624)
(connected strategic_location_4 strategic_location_5)  (= (distance strategic_location_4 strategic_location_5) 2)
(connected strategic_location_4 wp_AUV0)  (= (distance strategic_location_4 wp_AUV0) 74.0675)
(connected strategic_location_4 strategic_location_8)  (= (distance strategic_location_4 strategic_location_8) 14.7054)
(connected strategic_location_4 strategic_location_2)  (= (distance strategic_location_4 strategic_location_2) 18.4391)
(connected strategic_location_4 strategic_location_7)  (= (distance strategic_location_4 strategic_location_7) 11.6726)
(connected strategic_location_4 mission_site_start_point_1)  (= (distance strategic_location_4 mission_site_start_point_1) 51.6624)
(connected strategic_location_4 strategic_location_5)  (= (distance strategic_location_4 strategic_location_5) 2)
(connected strategic_location_8 strategic_location_6)  (= (distance strategic_location_8 strategic_location_6) 11.6726)
(connected strategic_location_8 strategic_location_1)  (= (distance strategic_location_8 strategic_location_1) 13.7931)
(connected strategic_location_8 strategic_location_3)  (= (distance strategic_location_8 strategic_location_3) 15.3052)
(connected strategic_location_8 strategic_location_4)  (= (distance strategic_location_8 strategic_location_4) 14.7054)
(connected strategic_location_8 strategic_location_8)  (= (distance strategic_location_8 strategic_location_8) 0)
(connected strategic_location_8 strategic_location_8)  (= (distance strategic_location_8 strategic_location_8) 0)
(connected strategic_location_8 strategic_location_2)  (= (distance strategic_location_8 strategic_location_2) 12.4197)
(connected strategic_location_8 strategic_location_7)  (= (distance strategic_location_8 strategic_location_7) 20)
(connected strategic_location_8 mission_site_start_point_1)  (= (distance strategic_location_8 mission_site_start_point_1) 37.2055)
(connected strategic_location_8 strategic_location_5)  (= (distance strategic_location_8 strategic_location_5) 13.1244)
(connected strategic_location_8 wp_AUV0)  (= (distance strategic_location_8 wp_AUV0) 59.5924)
(connected strategic_location_8 strategic_location_2)  (= (distance strategic_location_8 strategic_location_2) 12.4197)
(connected strategic_location_8 strategic_location_7)  (= (distance strategic_location_8 strategic_location_7) 20)
(connected strategic_location_8 mission_site_start_point_1)  (= (distance strategic_location_8 mission_site_start_point_1) 37.2055)
(connected strategic_location_8 strategic_location_5)  (= (distance strategic_location_8 strategic_location_5) 13.1244)
(connected strategic_location_2 strategic_location_6)  (= (distance strategic_location_2 strategic_location_6) 18)
(connected strategic_location_2 strategic_location_1)  (= (distance strategic_location_2 strategic_location_1) 2)
(connected strategic_location_2 strategic_location_3)  (= (distance strategic_location_2 strategic_location_3) 4)
(connected strategic_location_2 strategic_location_4)  (= (distance strategic_location_2 strategic_location_4) 18.4391)
(connected strategic_location_2 strategic_location_8)  (= (distance strategic_location_2 strategic_location_8) 12.4197)
(connected strategic_location_2 strategic_location_2)  (= (distance strategic_location_2 strategic_location_2) 0)
(connected strategic_location_2 strategic_location_2)  (= (distance strategic_location_2 strategic_location_2) 0)
(connected strategic_location_2 strategic_location_7)  (= (distance strategic_location_2 strategic_location_7) 15.3052)
(connected strategic_location_2 mission_site_start_point_1)  (= (distance strategic_location_2 mission_site_start_point_1) 40.3609)
(connected strategic_location_2 strategic_location_5)  (= (distance strategic_location_2 strategic_location_5) 18.1108)
(connected strategic_location_2 wp_AUV0)  (= (distance strategic_location_2 wp_AUV0) 62.6578)
(connected strategic_location_2 strategic_location_7)  (= (distance strategic_location_2 strategic_location_7) 15.3052)
(connected strategic_location_2 mission_site_start_point_1)  (= (distance strategic_location_2 mission_site_start_point_1) 40.3609)
(connected strategic_location_2 strategic_location_5)  (= (distance strategic_location_2 strategic_location_5) 18.1108)
(connected strategic_location_7 strategic_location_6)  (= (distance strategic_location_7 strategic_location_6) 14.7054)
(connected strategic_location_7 strategic_location_1)  (= (distance strategic_location_7 strategic_location_1) 13.7931)
(connected strategic_location_7 strategic_location_3)  (= (distance strategic_location_7 strategic_location_3) 12.4197)
(connected strategic_location_7 strategic_location_4)  (= (distance strategic_location_7 strategic_location_4) 11.6726)
(connected strategic_location_7 strategic_location_8)  (= (distance strategic_location_7 strategic_location_8) 20)
(connected strategic_location_7 strategic_location_2)  (= (distance strategic_location_7 strategic_location_2) 15.3052)
(connected strategic_location_7 strategic_location_7)  (= (distance strategic_location_7 strategic_location_7) 0)
(connected strategic_location_7 strategic_location_7)  (= (distance strategic_location_7 strategic_location_7) 0)
(connected strategic_location_7 mission_site_start_point_1)  (= (distance strategic_location_7 mission_site_start_point_1) 54.6283)
(connected strategic_location_7 strategic_location_5)  (= (distance strategic_location_7 strategic_location_5) 13.1244)
(connected strategic_location_7 wp_AUV0)  (= (distance strategic_location_7 wp_AUV0) 77.1443)
(connected strategic_location_7 mission_site_start_point_1)  (= (distance strategic_location_7 mission_site_start_point_1) 54.6283)
(connected strategic_location_7 strategic_location_5)  (= (distance strategic_location_7 strategic_location_5) 13.1244)
(connected mission_site_start_point_1 strategic_location_6)  (= (distance mission_site_start_point_1 strategic_location_6) 48.4665)
(connected mission_site_start_point_1 strategic_location_1)  (= (distance mission_site_start_point_1 strategic_location_1) 42.2493)
(connected mission_site_start_point_1 strategic_location_3)  (= (distance mission_site_start_point_1 strategic_location_3) 44.1475)
(connected mission_site_start_point_1 strategic_location_4)  (= (distance mission_site_start_point_1 strategic_location_4) 51.6624)
(connected mission_site_start_point_1 strategic_location_8)  (= (distance mission_site_start_point_1 strategic_location_8) 37.2055)
(connected mission_site_start_point_1 strategic_location_2)  (= (distance mission_site_start_point_1 strategic_location_2) 40.3609)
(connected mission_site_start_point_1 strategic_location_7)  (= (distance mission_site_start_point_1 strategic_location_7) 54.6283)
(connected mission_site_start_point_1 mission_site_start_point_1)  (= (distance mission_site_start_point_1 mission_site_start_point_1) 0)
(connected mission_site_start_point_1 mission_site_start_point_1)  (= (distance mission_site_start_point_1 mission_site_start_point_1) 0)
(connected mission_site_start_point_1 strategic_location_5)  (= (distance mission_site_start_point_1 strategic_location_5) 50.05)
(connected mission_site_start_point_1 wp_AUV0)  (= (distance mission_site_start_point_1 wp_AUV0) 22.561)
(connected mission_site_start_point_1 strategic_location_5)  (= (distance mission_site_start_point_1 strategic_location_5) 50.05)
(connected strategic_location_5 strategic_location_6)  (= (distance strategic_location_5 strategic_location_6) 2)
(connected strategic_location_5 strategic_location_1)  (= (distance strategic_location_5 strategic_location_1) 18)
(connected strategic_location_5 strategic_location_3)  (= (distance strategic_location_5 strategic_location_3) 18.1108)
(connected strategic_location_5 strategic_location_4)  (= (distance strategic_location_5 strategic_location_4) 2)
(connected strategic_location_5 strategic_location_8)  (= (distance strategic_location_5 strategic_location_8) 13.1244)
(connected strategic_location_5 strategic_location_2)  (= (distance strategic_location_5 strategic_location_2) 18.1108)
(connected strategic_location_5 strategic_location_7)  (= (distance strategic_location_5 strategic_location_7) 13.1244)
(connected strategic_location_5 mission_site_start_point_1)  (= (distance strategic_location_5 mission_site_start_point_1) 50.05)
(connected strategic_location_5 strategic_location_5)  (= (distance strategic_location_5 strategic_location_5) 0)
(connected strategic_location_5 strategic_location_5)  (= (distance strategic_location_5 strategic_location_5) 0)
(connected strategic_location_5 wp_AUV0)  (= (distance strategic_location_5 wp_AUV0) 72.4017)
)
(:goal (and
(>= (observed inspection_point_24) 1)
(>= (observed inspection_point_25) 1)
(>= (observed inspection_point_26) 1)
(>= (observed inspection_point_27) 1)
(>= (observed inspection_point_28) 1)
(>= (observed inspection_point_29) 1)
(>= (observed inspection_point_30) 1)
(>= (observed inspection_point_31) 1)
(near auv0 mission_site_start_point_1)
)))