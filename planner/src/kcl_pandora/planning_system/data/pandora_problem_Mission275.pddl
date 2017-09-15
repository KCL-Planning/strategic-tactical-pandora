(define (problem pandora_mission_task)
(:domain pandora_domain_persistent)
(:objects
auv0 - vehicle
inspection_point_166 - inspectionpoint
mission_site_start_point_3 strategic_location_1 wp_AUV0 - Waypoint
)
(:init
(= (arm_calibration auv0) 0)
(not_illuminating auv0)
(near auv0 wp_auv0)

(waypoint_not_occupied mission_site_start_point_3)
(waypoint_not_occupied strategic_location_1)
(waypoint_not_occupied wp_AUV0)

(= (observed inspection_point_166) 0)
(cansee auv0 strategic_location_1 inspection_point_166) (= (obs inspection_point_166 strategic_location_1) 1)
(cansee auv0 strategic_location_1 inspection_point_166) (= (obs inspection_point_166 strategic_location_1) 1)
(connected wp_AUV0 strategic_location_1)  (= (distance wp_AUV0 strategic_location_1) 83.666)
(connected wp_AUV0 mission_site_start_point_3)  (= (distance wp_AUV0 mission_site_start_point_3) 70.7743)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
(connected strategic_location_1 mission_site_start_point_3)  (= (distance strategic_location_1 mission_site_start_point_3) 40.8044)
(connected strategic_location_1 wp_AUV0)  (= (distance strategic_location_1 wp_AUV0) 83.666)
(connected strategic_location_1 mission_site_start_point_3)  (= (distance strategic_location_1 mission_site_start_point_3) 40.8044)
(connected mission_site_start_point_3 strategic_location_1)  (= (distance mission_site_start_point_3 strategic_location_1) 40.8044)
(connected mission_site_start_point_3 mission_site_start_point_3)  (= (distance mission_site_start_point_3 mission_site_start_point_3) 0)
(connected mission_site_start_point_3 mission_site_start_point_3)  (= (distance mission_site_start_point_3 mission_site_start_point_3) 0)
(connected mission_site_start_point_3 wp_AUV0)  (= (distance mission_site_start_point_3 wp_AUV0) 70.7743)
)
(:goal (and
(>= (observed inspection_point_166) 1)
(near auv0 mission_site_start_point_3)
)))
