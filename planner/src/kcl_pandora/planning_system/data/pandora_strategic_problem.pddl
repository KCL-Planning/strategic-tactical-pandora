(define (problem pandora_strategic_mission)
(:domain pandora_domain_strategic)
(:objects
auv - vehicle
mission_site_start_point_1 wp_auv0 - waypoint
Mission1 - mission
)
(:init
(vehicle_free auv)
(= (mission_total) 0)
(at auv wp_auv0) (= (charge auv) 1200)


(active Mission1)

(at 86400 (not (active Mission1)))

(in Mission1 mission_site_start_point_1)

(= (mission_duration Mission1) 336.17)

(connected mission_site_start_point_1 wp_auv0) (= (distance mission_site_start_point_1 wp_auv0) 22.561)
(connected wp_auv0 mission_site_start_point_1) (= (distance wp_auv0 mission_site_start_point_1) 22.561)

)
(:metric maximize (mission_total))
(:goal (> (mission_total) 0))
)
