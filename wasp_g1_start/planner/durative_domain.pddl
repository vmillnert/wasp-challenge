(define (domain emergency)

(:requirements :strips :typing :equality :durative-actions :fluents)

(:types 
  waypoint - object
  airwaypoint - waypoint 
  ;agent is supertype of turtlebot and drone
  agent - object
  turtlebot drone - agent
  person - object
  box - object)

(:predicates
  (at ?object - object ?waypoint - waypoint)

  (over ?airwaypoint - airwaypoint ?waypoint - waypoint)

  (handled ?person - person)

  (empty ?aw_object - (either waypoint agent))

  (carrying ?agent - agent ?box - box))

(:functions (move-duration ?from ?to - waypoint) - number)

(:durative-action move
  :parameters (?agent - agent ?from ?to - waypoint)
  :duration (= ?duration (move-duration ?from ?to))
  :condition (and (over all (empty ?to))
                  (at start (at ?agent ?from)))
  :effect (and (at end (not (at ?agent ?from)))
               (at end (at ?agent ?to))
               (at end (empty ?from))
               (at start (not (empty ?to))))
  )

;only drone can do pick-up, from a known waypoint
(:durative-action pick-up
  :parameters (?drone - drone ?box - box ?air - airwaypoint ?ground - waypoint)
  :duration (= ?duration 1)
  :condition (and (over all (over ?air ?ground))
                  (over all (at ?drone ?air))
                  (over all (at ?box ?ground))
                  (at start (empty ?drone)))
  :effect (and (at start (not (at ?box ?ground)))
               (at end (carrying ?drone ?box))
               (at end (not (empty ?drone))))
  )

(:durative-action hand-over
  :parameters (?drone - drone ?turtlebot - turtlebot ?box - box ?air - airwaypoint ?ground - waypoint)
  :duration (= ?duration 1)
  :condition (and (at start (over ?air ?ground))
                  (over all (at ?drone ?air))
                  (over all (at ?turtlebot ?ground))
                  (at start (carrying ?drone ?box))
                  (at start (empty ?turtlebot)))
  :effect (and (at end (not (carrying ?drone ?box)))
               (at end (empty ?drone))
               (at end (carrying ?turtlebot ?box))
               (at start (not (empty ?turtlebot))))
  )

(:durative-action deliver
  :parameters (?drone - drone ?box - box ?air - airwaypoint ?ground - waypoint ?person - person)
  :duration (= ?duration 1)
  :condition (and (over all (over ?air ?ground))
                  (over all (empty ?ground))
                  (at start (carrying ?drone ?box))
                  (over all (at ?drone ?air))
                  (over all (at ?person ?ground)))
  :effect (and (at end (not (carrying ?drone ?box)))
               (at end (empty ?drone))
               (at end (handled ?person)))
  )
)
