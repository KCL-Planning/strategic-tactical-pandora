(define (domain pandora)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :timed-initial-literals :duration-inequalities)

(:types
		waypoint
		inspectionpoint
		pillar
		panel
		valve
		chain
		vehicle)

(:predicates

	(waypoint_not_occupied ?wp - waypoint)
	(connected ?wp1 ?wp2 - waypoint)

	(at ?v - vehicle ?wp - waypoint)
	(near ?v - vehicle ?wp - waypoint)
	(not_illuminating ?v - vehicle)

	(cansee ?v - vehicle ?wp - waypoint ?ip - inspectionpoint)
	
	(cansee_pillar ?v - vehicle ?wp - waypoint ?p - pillar)
	(observed_pillar ?p - pillar)
	(pillar_illuminated ?p - pillar)
	
	(canexamine ?v - vehicle ?wp - waypoint ?p - panel)
	(canreach ?v - vehicle ?wp - waypoint ?p - panel)
	(examined ?p - panel)
	(on ?a - valve ?p - panel)
	(valve_blocked ?a - valve)
	(valve_free ?a - valve)

	(chainat ?c - chain ?wp - waypoint)
	(chain_examined ?c - chain)
)

(:functions

		(arm_calibration ?auv - vehicle)

		(observed ?ip - inspectionpoint)
		(obs ?ip - inspectionpoint ?wp - waypoint)

		(distance ?wp1 ?wp2 - waypoint)

		(valve_goal ?va - valve)
		(valve_state ?va - valve)
		(valve_goal_completed ?va - valve)
)


(:durative-action do_hover_controlled
	:parameters (?v - vehicle ?from ?to - waypoint)
	:duration ( = ?duration (* (distance ?from ?to) 2))
	:condition (and
				(at start (at ?v ?from)) 
;;				(over all (waypoint_not_occupied ?to))
				(at start (connected ?from ?to)))
	:effect (and 
				(at start (not (at ?v ?from)))
;;			    (at end (waypoint_not_occupied ?from))
;;				(at end (not (waypoint_not_occupied ?to)))
				(at end (at ?v ?to)))
)


(:durative-action do_hover_fast
	:parameters (?v - vehicle ?from ?to - waypoint)
	:duration ( = ?duration (* (distance ?from ?to) 1))
	:condition (and
				(at start (at ?v ?from))
;;				(over all (waypoint_not_occupied ?to))
				(at start (connected ?from ?to)))
	:effect (and
				(at start (not (at ?v ?from)))
;;				(at start (waypoint_not_occupied ?from))
;;				(at end (not (waypoint_not_occupied ?to)))
				(at end (near ?v ?to)))
)


(:durative-action correct_position
	:parameters (?v - vehicle ?target - waypoint)
	:duration ( = ?duration 3)
	:condition (and
				(at start (near ?v ?target)))
	:effect (and
				(at start (not (near ?v ?target)))
				(at end (at ?v ?target)))
)


(:durative-action observe_inspection_point
	:parameters (?v - vehicle ?wp - waypoint ?ip - inspectionpoint)
	:duration ( = ?duration 10)
	:condition (and
				(at start (at ?v ?wp))
				(at start (cansee ?v ?wp ?ip)))
	:effect (and 
				(at start (not (cansee ?v ?wp ?ip)))
				(at start (not (at ?v ?wp)))
				(at end (increase (observed ?ip) (obs ?ip ?wp)))
				(at end (near ?v ?wp)))
)

(:durative-action illuminate_pillar
	:parameters (?v - vehicle ?wp - waypoint ?p - pillar)
	:duration ( = ?duration 50)
	:condition (and
				(at start (at ?v ?wp))
				(at start (cansee_pillar ?v ?wp ?p)))
	:effect (and 				
				(at start (pillar_illuminated ?p))
				(at start (not (not_illuminating ?v)))
				(at start (not (at ?v ?wp)))
				(at end (not (pillar_illuminated ?p)))
				(at end (not_illuminating ?v))
				(at end (near ?v ?wp)))
)

(:durative-action observe_pillar
	:parameters (?v - vehicle ?wp - waypoint ?p - pillar)
	:duration ( = ?duration 10)
	:condition (and
				(at start (at ?v ?wp))
				(at start (cansee_pillar ?v ?wp ?p))
				(at start (not_illuminating ?v))
				(over all (pillar_illuminated ?p)))
	:effect (and 
				(at start (not (cansee_pillar ?v ?wp ?p)))
				(at start (not (at ?v ?wp)))
				(at end (observed_pillar ?p))
				(at end (near ?v ?wp)))
)

(:durative-action examine_panel
	:parameters (?v - vehicle ?wp - waypoint ?p - panel)
	:duration ( = ?duration 10)
	:condition (and
				(at start (at ?v ?wp))
				(at start (canexamine ?v ?wp ?p)))
	:effect (and
				(at start (not (canexamine ?v ?wp ?p)))
				(at end (examined ?p)))
)

(:durative-action turn_valve
	:parameters (?v - vehicle ?wp - waypoint ?p - panel ?a - valve)
	:duration ( = ?duration 120)
	:condition (and 
				(at start (at ?v ?wp))
;				(at start (examined ?p))
				(at start (canreach ?v ?wp ?p))
				(at start (on ?a ?p))
				(at start (<= (arm_calibration ?v) 1))
				(at start (valve_free ?a)))
	:effect (and
				(at start (not (valve_free ?a)))
				(at end (assign (valve_state ?a) (valve_goal ?a)))
				(at start (not (at ?v ?wp)))
				(at end (near ?v ?wp))
				(at end (increase (valve_goal_completed ?a) 1))
;				(at start (not (examined ?p)))
				(at end (increase (arm_calibration ?v) 1))
				(at end (valve_blocked ?a)))
)

;(:durative-action recalibrate_arm
;	:parameters (?v - vehicle ?wp - waypoint)
;	:duration ( = ?duration 60)
;	:condition (and 
;				(at start (>= (arm_calibration ?v) 1)))
;	:effect (and
;				(at end (assign (arm_calibration ?v) 0)))
;)

(:durative-action follow_chain
	:parameters (?v - vehicle ?c - chain ?from - waypoint)
	:duration ( = ?duration 90)
	:condition (and
				(at start (at ?v ?from))
				(at start (chainat ?c ?from)))
	:effect (and
				(at start (not (at ?v ?from)))
				(at end (chain_examined ?c))
				(at end (near ?v ?from)))
)

)
