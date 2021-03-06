(define (problem pandora_strategic_mission)
(:domain pandora_domain_strategic)
(:objects
auv - vehicle
mission_site_start_point_0 mission_site_start_point_1 mission_site_start_point_2 mission_site_start_point_3 wp_auv0 - waypoint
Mission0 Mission1 Mission10 Mission11 Mission12 Mission13 Mission14 Mission15 Mission16 Mission17 Mission18 Mission19 Mission2 Mission20 Mission21 Mission22 Mission23 Mission24 Mission3 Mission4 Mission5 Mission6 Mission7 Mission8 Mission9 - mission
)
(:init
(vehicle_free auv)
(= (mission_total) 0)
(at auv wp_auv0) (= (charge auv) 1200)

(recharge_at mission_site_start_point_0)

(active Mission0)
(active Mission1)
(active Mission10)
(active Mission11)
(active Mission12)
(active Mission13)
(active Mission14)
(active Mission15)
(active Mission16)
(active Mission17)
(active Mission18)
(active Mission19)
(active Mission2)
(active Mission20)
(active Mission21)
(active Mission22)
(active Mission23)
(active Mission24)
(active Mission3)
(active Mission4)
(active Mission5)
(active Mission6)
(active Mission7)
(active Mission8)
(active Mission9)

(at 4100 (not (active Mission0)))
(at 7100 (not (active Mission1)))
(at 86400 (not (active Mission10)))
(at 86400 (not (active Mission11)))
(at 86400 (not (active Mission12)))
(at 86400 (not (active Mission13)))
(at 86400 (not (active Mission14)))
(at 86400 (not (active Mission15)))
(at 86400 (not (active Mission16)))
(at 86400 (not (active Mission17)))
(at 86400 (not (active Mission18)))
(at 86400 (not (active Mission19)))
(at 86400 (not (active Mission2)))
(at 86400 (not (active Mission20)))
(at 86400 (not (active Mission21)))
(at 86400 (not (active Mission22)))
(at 86400 (not (active Mission23)))
(at 86400 (not (active Mission24)))
(at 86400 (not (active Mission3)))
(at 86400 (not (active Mission4)))
(at 86400 (not (active Mission5)))
(at 86400 (not (active Mission6)))
(at 86400 (not (active Mission7)))
(at 86400 (not (active Mission8)))
(at 86400 (not (active Mission9)))

(in Mission0 mission_site_start_point_1)
(in Mission1 mission_site_start_point_1)
(in Mission10 mission_site_start_point_2)
(in Mission11 mission_site_start_point_2)
(in Mission12 mission_site_start_point_2)
(in Mission13 mission_site_start_point_2)
(in Mission14 mission_site_start_point_2)
(in Mission15 mission_site_start_point_2)
(in Mission16 mission_site_start_point_2)
(in Mission17 mission_site_start_point_2)
(in Mission18 mission_site_start_point_2)
(in Mission19 mission_site_start_point_2)
(in Mission2 mission_site_start_point_1)
(in Mission20 mission_site_start_point_3)
(in Mission21 mission_site_start_point_3)
(in Mission22 mission_site_start_point_3)
(in Mission23 mission_site_start_point_3)
(in Mission24 mission_site_start_point_3)
(in Mission3 mission_site_start_point_1)
(in Mission4 mission_site_start_point_1)
(in Mission5 mission_site_start_point_1)
(in Mission6 mission_site_start_point_1)
(in Mission7 mission_site_start_point_1)
(in Mission8 mission_site_start_point_1)
(in Mission9 mission_site_start_point_1)

(= (mission_duration Mission0) 261.868)
(= (mission_duration Mission1) 242.065)
(= (mission_duration Mission10) 138.866)
(= (mission_duration Mission11) 32.561)
(= (mission_duration Mission12) 336.17)
(= (mission_duration Mission13) 365.49)
(= (mission_duration Mission14) 340.969)
(= (mission_duration Mission15) 337.334)
(= (mission_duration Mission16) 340.969)
(= (mission_duration Mission17) 337.334)
(= (mission_duration Mission18) 1020.3)
(= (mission_duration Mission19) 1230.53)
(= (mission_duration Mission2) 336.17)
(= (mission_duration Mission20) 336.17)
(= (mission_duration Mission21) 80.774)
(= (mission_duration Mission22) 384.45)
(= (mission_duration Mission23) 1058.18)
(= (mission_duration Mission24) 1263.43)
(= (mission_duration Mission3) 365.49)
(= (mission_duration Mission4) 340.969)
(= (mission_duration Mission5) 337.334)
(= (mission_duration Mission6) 340.969)
(= (mission_duration Mission7) 337.334)
(= (mission_duration Mission8) 1020.3)
(= (mission_duration Mission9) 1230.53)

(connected mission_site_start_point_0 mission_site_start_point_1) (= (distance mission_site_start_point_0 mission_site_start_point_1) 53.8888)
(connected mission_site_start_point_0 mission_site_start_point_2) (= (distance mission_site_start_point_0 mission_site_start_point_2) 53.8888)
(connected mission_site_start_point_0 mission_site_start_point_3) (= (distance mission_site_start_point_0 mission_site_start_point_3) 58.3438)
(connected mission_site_start_point_0 wp_auv0) (= (distance mission_site_start_point_0 wp_auv0) 56.7891)
(connected wp_auv0 mission_site_start_point_0) (= (distance wp_auv0 mission_site_start_point_0) 56.7891)
(connected mission_site_start_point_1 mission_site_start_point_0) (= (distance mission_site_start_point_1 mission_site_start_point_0) 53.8888)
(connected mission_site_start_point_1 mission_site_start_point_2) (= (distance mission_site_start_point_1 mission_site_start_point_2) 0)
(connected mission_site_start_point_1 mission_site_start_point_3) (= (distance mission_site_start_point_1 mission_site_start_point_3) 50)
(connected mission_site_start_point_1 wp_auv0) (= (distance mission_site_start_point_1 wp_auv0) 22.561)
(connected wp_auv0 mission_site_start_point_1) (= (distance wp_auv0 mission_site_start_point_1) 22.561)
(connected mission_site_start_point_2 mission_site_start_point_0) (= (distance mission_site_start_point_2 mission_site_start_point_0) 53.8888)
(connected mission_site_start_point_2 mission_site_start_point_1) (= (distance mission_site_start_point_2 mission_site_start_point_1) 0)
(connected mission_site_start_point_2 mission_site_start_point_3) (= (distance mission_site_start_point_2 mission_site_start_point_3) 50)
(connected mission_site_start_point_2 wp_auv0) (= (distance mission_site_start_point_2 wp_auv0) 22.561)
(connected wp_auv0 mission_site_start_point_2) (= (distance wp_auv0 mission_site_start_point_2) 22.561)
(connected mission_site_start_point_3 mission_site_start_point_0) (= (distance mission_site_start_point_3 mission_site_start_point_0) 58.3438)
(connected mission_site_start_point_3 mission_site_start_point_1) (= (distance mission_site_start_point_3 mission_site_start_point_1) 50)
(connected mission_site_start_point_3 mission_site_start_point_2) (= (distance mission_site_start_point_3 mission_site_start_point_2) 50)
(connected mission_site_start_point_3 wp_auv0) (= (distance mission_site_start_point_3 wp_auv0) 70.7743)
(connected wp_auv0 mission_site_start_point_3) (= (distance wp_auv0 mission_site_start_point_3) 70.7743)

)
(:metric maximize (mission_total))
)
