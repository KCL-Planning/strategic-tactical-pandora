(define (problem pandora_mission_task)
(:domain pandora_domain_persistent)
(:objects
auv0 - vehicle
- Waypoint
)
(:init
(= (arm_calibration auv0) 0)
(not_illuminating auv0)
)
(:goal (and
(near auv0 mission_site_start_point_1)
)))
