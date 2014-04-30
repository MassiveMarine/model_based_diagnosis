(define (domain repair_domain)
   (:requirements :strips :equality :typing :negative-preconditions)
   (:types hardware)
   (:predicates
	 (bad ?c)
	 (good ?c)
	 (running ?c)
         (ok ?c)
	 (on ?o)
    )

   (:action power_up
    :parameters (?c)
                :precondition (and (not(on ?c))(bad ?c))
                :effect (and (on ?c)(good ?c))
    )

   (:action shutdown
    :parameters (?c)
                :precondition (and (on ?c)(bad ?c))
                :effect (and (bad ?c)(not(on ?c)))
    )

   (:action stop_node
    :parameters (?c)
                :precondition (and (bad ?c)(running ?c))
                :effect (and (bad ?c)(not(running ?c)))
    )

   (:action start_node
    :parameters (?c)
         	:precondition (and (bad ?c)(not(running ?c)))
                :effect (good ?c)
    )
)

