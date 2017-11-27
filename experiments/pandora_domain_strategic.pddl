(define (domain pandora_domain_strategic)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :timed-initial-literals)

(:types
		waypoint
		mission
		vehicle)

(:predicates

	(connected ?wp1 ?wp2 - waypoint)
	(at ?v - vehicle ?wp - waypoint)
	(vehicle_free ?v - vehicle)

	(in ?m - mission ?wp - waypoint)
	(completed ?m - mission)
	(active ?m - mission)

	(recharge_at ?wp - waypoint)
	(docked ?v - vehicle ?wp - waypoint)
	(ready_to_undock ?v - vehicle ?wp - waypoint)
)

(:functions
	(distance ?wp1 ?wp2 - waypoint)
	(mission_duration ?m - mission)
	(charge ?v - vehicle)
	(mission_total)
)

(:durative-action do_hover
	:parameters (?v - vehicle ?from ?to - waypoint)
	:duration ( = ?duration (* (distance ?from ?to) 5))
	:condition (and
		(over all (vehicle_free ?v))
		(at start (at ?v ?from))
		(at start (>= (charge ?v) (* (distance ?from ?to) 5)))
		(at start (connected ?from ?to))
	)
	:effect (and
		(at start (not (at ?v ?from)))
		(at end (decrease (charge ?v) (* (distance ?from ?to) 5)))
		(at end (at ?v ?to))
	)
)

(:durative-action complete_mission
	:parameters (?v - vehicle ?m - mission ?wp - waypoint)
	:duration ( = ?duration (mission_duration ?m))
	:condition (and
		(over all (vehicle_free ?v))
		(over all (active ?m))
		(at start (in ?m ?wp))
		(at start (at ?v ?wp))
		(at start (>= (charge ?v) (mission_duration ?m)))
	)
	:effect (and
		(at start (not (at ?v ?wp)))
		(at end (increase (mission_total) 1))
		(at end (decrease (charge ?v) (mission_duration ?m)))
		(at end (completed ?m))
		(at end (at ?v ?wp))
	)
)

(:durative-action dock_auv
	:parameters (?v - vehicle ?wp - waypoint)
	:duration ( = ?duration 20)
	:condition (and
		(over all (at ?v ?wp))
		(at start (vehicle_free ?v))
		(at start (recharge_at ?wp))
		(at start (>= (charge ?v) 20))
	)
	:effect (and
		(at start (not (vehicle_free ?v)))
		(at end (docked ?v ?wp))
	)
)

(:durative-action recharge
	:parameters (?v - vehicle ?wp - waypoint)
	:duration ( = ?duration 1800)
	:condition (and
		(over all (docked ?v ?wp))
	)
	:effect (and
		(at end (assign (charge ?v) 1200))
		(at end (ready_to_undock ?v ?wp))
	)
)

(:durative-action undock_auv
	:parameters (?v - vehicle ?wp - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (at ?v ?wp))
		(at start (recharge_at ?wp))
		(at start (docked ?v ?wp))
		(at start (ready_to_undock ?v ?wp))
	)
	:effect (and
		(at start (not (ready_to_undock ?v ?wp)))
		(at start (not (docked ?v ?wp)))
		(at end (vehicle_free ?v))
	)
)
)
