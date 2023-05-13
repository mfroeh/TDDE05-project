(define (domain office)

(:requirements :strips :typing )

(:types
    person
    vending
    office
    user
    content
    robot
    person vending office user - location
)
(:predicates
    (robotAt ?r - robot ?l - location)
    (robotEmpty ?r - robot)
    (vendingHas ?v - vending ?c - content)
    (personHas ?p - person ?c - content)
    (personNeed ?p - person ?c - content)
    (robotHas ?r - robot ?c - content)
   ; (unvisited ?l - location)
    (visited ?r - robot ?l - location)
    (explored ?l - location)
)

(:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition(robotAt ?r ?from)
    :effect (and 
        (not (robotAt ?r ?from))
        (robotAt ?r ?to)
    )
)

(:action getContent
    :parameters (?r - robot ?from - location ?v - vending ?c - content)
    :precondition (and
        (robotAt ?r ?from)
        (vendingHas ?v ?c)
        (robotEmpty ?r)
     )
    :effect (and
        (not (robotEmpty ?r))
        (not (robotAt ?r ?from))
        (robotAt ?r ?v)
        (robotHas ?r ?c)
     )
)

(:action giveContent
    :parameters (?r - robot ?from - location ?p - person ?c - content)
    :precondition (and
        (robotAt ?r ?from)
        (robotHas ?r ?c)
        (personNeed ?p ?c)
     )
    :effect (and
        (robotEmpty ?r)
        (not (robotHas ?r ?c))
        (not (robotAt ?r ?from))
        (robotAt ?r ?p)
        (personHas ?p ?c)
        (not (personNeed ?p ?c))
     )
)

(:action visit
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
        (robotAt ?r ?from)
      ;  (unvisited ?to)
    )
    :effect (and
        (not (robotAt ?r ?from))
      ;  (not (unvisited ?to))
        (robotAt ?r ?to)
        (visited ?r ?to)
     )
)


)

