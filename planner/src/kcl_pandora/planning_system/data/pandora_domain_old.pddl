(define (domain pandora_valve_turning)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
		waypoint 
		inspectionpoint
		panel
		valve
		vehicle)

(:predicates
	(connected ?wp1 ?wp2 - waypoint)
	(at ?v - vehicle ?wp - waypoint)
	(near ?v - vehicle ?wp - waypoint)
	(cansee ?v - vehicle ?ip - inspectionpoint ?wp - waypoint)
	(canexamine ?v - vehicle ?p - panel ?wp - waypoint)
	(canreach ?v - vehicle ?p - panel ?wp - waypoint)
	(examined ?p - panel)
	(on ?a - valve ?p - panel)
	(turned ?a - valve)
	(chainat ?wp - waypoint)
)

(:functions
		(observed ?ip - inspectionpoint)
		(obs ?ip - inspectionpoint ?wp - waypoint)
		(distance ?wp1 ?wp2 - waypoint) 
)

(:durative-action do_hover
	:parameters (?v - vehicle ?from ?to - waypoint)
	:duration ( = ?duration (* (distance ?from ?to) 10))
	:condition (and (at start (at ?v ?from)) (at start (connected ?from ?to)))
	:effect (and (at start (not (at ?v ?from))) (at end (at ?v ?to)))
)

(:durative-action do_hover_fast
	:parameters (?v - vehicle ?from ?to - waypoint)
	:duration ( = ?duration (* (distance ?from ?to) 5))
	:condition (and (at start (at ?v ?from)) (at start (connected ?from ?to)))
	:effect (and (at start (not (at ?v ?from))) (at end (near ?v ?to)))
)

(:durative-action correct_position
	:parameters (?v - vehicle ?target - waypoint)
	:duration ( = ?duration 10)
	:condition (at start (near ?v ?target))
	:effect (and (at start (not (near ?v ?target))) (at end (at ?v ?target)))
)

(:durative-action valve_state
	:parameters (?v - vehicle ?wp - waypoint ?p - panel)
	:duration ( = ?duration 10)
	:condition (and (at start (at ?v ?wp)) (at start (canexamine ?v ?p ?wp)))
	:effect (and
			(at start (not (canexamine ?v ?p ?wp)))
			(at end (examined ?p)))
)

(:durative-action observe
	:parameters (?v - vehicle ?wp - waypoint ?ip - inspectionpoint)
	:duration ( = ?duration 10)
	:condition (and (at start (at ?v ?wp)) (at start (cansee ?v ?ip ?wp)))
	:effect (and 
			(and (at start (not (cansee ?v ?ip ?wp))) (at end (increase (observed ?ip) (obs ?ip ?wp))))
			(and (at start (not (at ?v ?wp))) (at end (near ?v ?wp))))
)

(:durative-action turn_valve
	:parameters (?v - vehicle ?wp - waypoint ?p - panel ?a - valve)
	:duration ( = ?duration 30)
	:condition (and 
			(and (at start (at ?v ?wp)) (at start (canreach ?v ?p ?wp)))
			(at start (on ?a ?p)))
	:effect (and 
			(at end (turned ?a)) 
			(and (at start (not (at ?v ?wp))) (at end (near ?v ?wp))))
)

(:durative-action follow_chain
	:parameters (?v - vehicle ?from - waypoint)
	:duration ( = ?duration 120)
	:condition (and (at start (at ?v ?from)) (at start (chainat ?from)))
	:effect (and (at start (not (at ?v ?from))) (at end (near ?v ?from)))
)
)
