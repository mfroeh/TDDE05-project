(define (domain office)

(:requirements :strips :typing )

(:types
    person
    vending
    office
    user
    content
    robot
    location - object
    person vending office user - location
)
(:predicates
    (robotat ?r - robot ?l - location)
    (robotempty ?r - robot)
    (vendinghas ?v - vending ?c - content)
    (personhas ?p - person ?c - content)
    (personneed ?p - person ?c - content)
    (robothas ?r - robot ?c - content)
    (visited ?r - robot ?l - location)
    (unvisited ?location)
)



(:action getcontent
    :parameters (?r - robot ?from - location ?v - vending ?c - content)
    :precondition (and
        (robotat ?r ?from)
        (vendinghas ?v ?c)
        (robotempty ?r)
     )
    :effect (and
        (not (robotempty ?r))
        (not (robotat ?r ?from))
        (robotat ?r ?v)
        (robothas ?r ?c)
     )
)

(:action givecontent
    :parameters (?r - robot ?from - location ?p - person ?c - content)
    :precondition (and
        (robotat ?r ?from)
        (robothas ?r ?c)
        (personneed ?p ?c)
     )
    :effect (and
        (robotempty ?r)
        (not (robothas ?r ?c))
        (not (robotat ?r ?from))
        (robotat ?r ?p)
        (personhas ?p ?c)
        (not (personneed ?p ?c))
     )
)

(:action visit
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
        (robotat ?r ?from)
        (unvisited ?to)
    )
    :effect (and
        (not (robotat ?r ?from))
        (not (unvisited ?to))
        (robotat ?r ?to)
        (visited ?r ?to)
     )
)


)

