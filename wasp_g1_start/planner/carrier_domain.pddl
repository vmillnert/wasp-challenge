(define (domain d_rescue)
  (:requirements :strips :equality :typing :adl :action-costs)
  (:types DRONE LOC BOX PERSON BTYPE CAPACITY_NBR CARRIER)
  (:predicates
    ; Static
    (empty ?d - DRONE)
    ; Dynamic
    (contains ?b - BOX ?t - BTYPE)
    (at ?o - (either CARRIER DRONE BOX PERSON) ?l - (either LOC DRONE PERSON CARRIER))
    (has ?p - PERSON ?t - BTYPE)
    (capacity ?c - CARRIER ?cn - CAPACITY_NBR)
    (pred ?low ?high - CAPACITY_NBR) ; capacity predecessor
  )

  (:functions (total-cost) - number
	            (fly-cost ?from ?to - LOC) - number
  )

  (:action fly
    :parameters (?d - DRONE ?from ?to - LOC)
    :precondition (at ?d ?from)
    :effect (and (not (at ?d ?from)) (at ?d ?to)
                 (increase (total-cost) (fly-cost ?from ?to)))
  )

  (:action pick_up
    :parameters (?d - DRONE ?l - LOC ?b - BOX)
    :precondition (and (at ?d ?l) (at ?b ?l) (empty ?d))
    :effect (and (not (at ?b ?l)) (at ?b ?d) (not (empty ?d))
                 (increase (total-cost) 10))
  )

  (:action drop
    :parameters (?d - DRONE ?l - LOC ?p - PERSON ?b - BOX ?t - BTYPE)
    :precondition (and (at ?d ?l) (at ?b ?d) (at ?p ?l) (contains ?b ?t))
    :effect (and (not (at ?b ?d)) (at ?b ?p) (has ?p ?t) (empty ?d)
                 (increase (total-cost) 10))
  )
  (:action fly_carrier
    :parameters (?d - DRONE ?c - CARRIER ?from ?to - LOC)
    :precondition (and (at ?d ?from) (at ?c ?from) (empty ?d))
    :effect (and (not (at ?d ?from)) (at ?d ?to) (not (at ?c ?from)) (at ?c ?to)
                 (increase (total-cost) (fly-cost ?from ?to)))
  )

  (:action pick_up_from_carrier
    :parameters (?d - DRONE ?c - CARRIER ?l - LOC ?b - BOX ?c_now ?c_new - CAPACITY_NBR)
    :precondition (and (at ?d ?l) (at ?c ?l) (at ?b ?c)
                       (empty ?d) (pred ?c_now ?c_new) (capacity ?c ?c_now))
    :effect (and (not (at ?b ?c)) (at ?b ?d) (not (empty ?d)) (capacity ?c ?c_new) (not (capacity ?c ?c_now))
                 (increase (total-cost) 10))
  )

  (:action drop_on_carrier
    :parameters (?d - DRONE ?c - CARRIER ?l - LOC ?b - BOX ?c_now ?c_new - CAPACITY_NBR)
    :precondition (and (at ?d ?l) (at ?c ?l) (at ?b ?d)
                       (pred ?c_new ?c_now) (capacity ?c ?c_now))
    :effect (and (not (at ?b ?d)) (at ?b ?c) (empty ?d) (capacity ?c ?c_new) (not (capacity ?c ?c_now))
                 (increase (total-cost) 10))
  )

)
