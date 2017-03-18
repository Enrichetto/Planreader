(define 
	(domain SofAR)
	(:types
		drone
		location
	)
	(:predicates
		(is-free ?l - location)
		(is-in ?d - drone ?l - location)
		(connected ?l1 - location ?l2 - location)
	)
	
	
	(:action move
		:parameters (?d - drone ?l1 - location ?l2 - location)
		:precondition (and 
			(is-in ?d ?l1)
			(connected ?l1 ?l2)
			(is-free ?l2))
		:effect (and 
				(is-in ?d ?l2)
				(not (is-in ?d ?l1))
				(is-free ?l1)
				(not (is-free ?l2)))
	)
)
