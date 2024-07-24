(define (domain draw_ai)
  (:requirements :strips :typing :action-costs)

  (:types 
    location turtle
  )

  (:predicates
    (at ?turtle - turtle ?loc - location)
    (pen-enabled ?turtle - turtle)
    (marked ?loc1 - location ?loc2 - location)
    (permissible-marking ?loc1 - location ?loc2 - location)
  )

  (:functions 
    (total-cost)
  )

  (:action move-with-pen
    :parameters (?turtle - turtle ?from - location ?to - location)
    :precondition (and (at ?turtle ?from) 
                       (pen-enabled ?turtle)
                       (permissible-marking ?from ?to))
    :effect (and 
              (not (at ?turtle ?from)) 
              (at ?turtle ?to)
              (marked ?from ?to)
              (marked ?to ?from)
              (increase (total-cost) 1)
            )
  )

  (:action move-without-pen
    :parameters (?turtle - turtle ?from - location ?to - location)
    :precondition (and (at ?turtle ?from) (not (pen-enabled ?turtle)))
    :effect (and 
              (not (at ?turtle ?from)) 
              (at ?turtle ?to)
              (increase (total-cost) 1)
            )
  )
  
  (:action enable-pen
    :parameters (?turtle - turtle)
    :precondition (not (pen-enabled ?turtle))
    :effect (pen-enabled ?turtle)
  )

  (:action disable-pen
    :parameters (?turtle - turtle)
    :precondition (pen-enabled ?turtle)
    :effect (not (pen-enabled ?turtle))
  )
)

