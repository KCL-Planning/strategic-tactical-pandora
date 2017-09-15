(define (problem pandora_mission_task)
(:domain pandora_domain_persistent)
(:objects
auv0 - vehicle
inspection_point_110 - inspectionpoint
mission_site_start_point_2 strategic_location_1 wp_AUV0 - Waypoint
)
(:init
(= (arm_calibration auv0) 0)
(not_illuminating auv0)
(near auv0 wp_auv0)

(waypoint_not_occupied mission_site_start_point_2)
(waypoint_not_occupied strategic_location_1)
(waypoint_not_occupied wp_AUV0)

(= (observed inspection_point_110) 0)
(cansee auv0 strategic_location_1 inspection_point_110) (= (obs inspection_point_110 strategic_location_1) 1)
(cansee auv0 strategic_location_1 inspection_point_110) (= (obs inspection_point_110 strategic_location_1) 1)
(connected wp_AUV0 mission_site_start_point_2)  (= (distance wp_AUV0 mission_site_start_point_2) 22.561)
(connected wp_AUV0 strategic_location_1)  (= (distance wp_AUV0 strategic_location_1) 81.8046)
(connected mission_site_start_point_2 mission_site_start_point_2)  (= (distance mission_site_start_point_2 mission_site_start_point_2) 0)
(connected mission_site_start_point_2 mission_site_start_point_2)  (= (distance mission_site_start_point_2 mission_site_start_point_2) 0)
(connected mission_site_start_point_2 strategic_location_1)  (= (distance mission_site_start_point_2 strategic_location_1) 68.8259)
(connected mission_site_start_point_2 wp_AUV0)  (= (distance mission_site_start_point_2 wp_AUV0) 22.561)
(connected mission_site_start_point_2 strategic_location_1)  (= (distance mission_site_start_point_2 strategic_location_1) 68.8259)
(connected strategic_location_1 mission_site_start_point_2)  (= (distance strategic_location_1 mission_site_start_point_2) 68.8259)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
(connected strategic_location_1 wp_AUV0)  (= (distance strategic_location_1 wp_AUV0) 81.8046)
)
(:goal (and
(>= (observed inspection_point_110) 1)
(near auv0 mission_site_start_point_2)
)))