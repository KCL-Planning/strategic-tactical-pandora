(define (problem pandora_mission_task)
(:domain pandora_domain_persistent)
(:objects
auv0 - vehicle
valve_panel_000 valve_panel_001 - panel
valve_000 valve_001 - valve
mission_site_start_point_1 strategic_location_1 strategic_location_2 wp_AUV0 - Waypoint
)
(:init
(= (arm_calibration auv0) 0)
(not_illuminating auv0)
(at auv0 wp_auv0)

(waypoint_not_occupied mission_site_start_point_1)
(waypoint_not_occupied strategic_location_1)
(waypoint_not_occupied strategic_location_2)
(waypoint_not_occupied wp_AUV0)

(on valve_000 valve_panel_000)
(= (valve_state valve_000) 0)
(= (valve_goal_completed valve_000) 0)
(valve_free valve_000)

(on valve_001 valve_panel_001)
(= (valve_state valve_001) 0)
(= (valve_goal_completed valve_001) 0)
(valve_free valve_001)

(canexamine auv0 strategic_location_1 valve_panel_000)
(canexamine auv0 strategic_location_2 valve_panel_001)
(canreach auv0 strategic_location_1 valve_panel_000)
(canreach auv0 strategic_location_2 valve_panel_001)
(connected wp_AUV0 mission_site_start_point_1)  (= (distance wp_AUV0 mission_site_start_point_1) 22.561)
(connected wp_AUV0 strategic_location_2)  (= (distance wp_AUV0 strategic_location_2) 55.9106)
(connected mission_site_start_point_1 mission_site_start_point_1)  (= (distance mission_site_start_point_1 mission_site_start_point_1) 0)
(connected mission_site_start_point_1 mission_site_start_point_1)  (= (distance mission_site_start_point_1 mission_site_start_point_1) 0)
(connected mission_site_start_point_1 strategic_location_2)  (= (distance mission_site_start_point_1 strategic_location_2) 50.1498)
(connected mission_site_start_point_1 strategic_location_1)  (= (distance mission_site_start_point_1 strategic_location_1) 50.1498)
(connected mission_site_start_point_1 wp_AUV0)  (= (distance mission_site_start_point_1 wp_AUV0) 22.561)
(connected mission_site_start_point_1 strategic_location_2)  (= (distance mission_site_start_point_1 strategic_location_2) 50.1498)
(connected mission_site_start_point_1 strategic_location_1)  (= (distance mission_site_start_point_1 strategic_location_1) 50.1498)
(connected strategic_location_2 mission_site_start_point_1)  (= (distance strategic_location_2 mission_site_start_point_1) 50.1498)
(connected strategic_location_2 strategic_location_2)  (= (distance strategic_location_2 strategic_location_2) 0)
(connected strategic_location_2 strategic_location_2)  (= (distance strategic_location_2 strategic_location_2) 0)
(connected strategic_location_2 strategic_location_1)  (= (distance strategic_location_2 strategic_location_1) 90)
(connected strategic_location_2 wp_AUV0)  (= (distance strategic_location_2 wp_AUV0) 55.9106)
(connected strategic_location_2 strategic_location_1)  (= (distance strategic_location_2 strategic_location_1) 90)
(connected strategic_location_1 mission_site_start_point_1)  (= (distance strategic_location_1 mission_site_start_point_1) 50.1498)
(connected strategic_location_1 strategic_location_2)  (= (distance strategic_location_1 strategic_location_2) 90)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
(connected strategic_location_1 strategic_location_1)  (= (distance strategic_location_1 strategic_location_1) 0)
)
(:goal (and
(examined valve_panel_000)
(>= (valve_goal_completed valve_000) 1)
(>= (valve_goal_completed valve_001) 1)
(near auv0 mission_site_start_point_1)
)))