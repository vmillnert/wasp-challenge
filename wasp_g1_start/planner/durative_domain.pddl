(define (domain emergency)

(:requirements :strips :typing :durative-actions :fluents)

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

  (carrying ?agent - agent ?box - box)

  (airborne ?drone - drone))

(:functions (move-duration ?from ?to - waypoint) - number)

(:durative-action fly
  :parameters (?agent - drone ?from ?to - waypoint)
  :duration (= ?duration (move-duration ?from ?to))
  :condition (and (over all (empty ?to))
                  (over all (empty ?agent))
                  (at start (at ?agent ?from))
                  (over all (airborne ?agent)))
  :effect (and (at start (not (at ?agent ?from)))
               (at end (at ?agent ?to))
               (at start (empty ?from))
               (at end (not (empty ?to))))
  )

(:durative-action goto
  :parameters (?agent - turtlebot ?from ?to - waypoint)
  :duration (= ?duration (move-duration ?from ?to))
  :condition (and (over all (empty ?to))
                  (at start (at ?agent ?from)))
  :effect (and (at start (not (at ?agent ?from)))
               (at end (at ?agent ?to))
               (at start (empty ?from))
               (at end (not (empty ?to))))
  )


;only drone can do pick-up, from a known waypoint
(:durative-action pick-up
  :parameters (?drone - drone ?box - box ?air - airwaypoint ?ground - waypoint)
  :duration (= ?duration 1)
  :condition (and (over all (over ?air ?ground))
                  (over all (at ?drone ?air))
                  (at start (at ?box ?ground))
                  (at start (empty ?drone))
                  (over all (airborne ?drone)))
  :effect (and (at start (not (at ?box ?ground)))
               (at end (carrying ?drone ?box))
               (at end (not (empty ?drone))))
  )

(:durative-action hand-over-drone2bot
  :parameters (?drone - drone ?turtlebot - turtlebot ?box - box ?air - airwaypoint ?ground - waypoint)
  :duration (= ?duration 1)
  :condition (and (over all (over ?air ?ground))
                  (over all (at ?drone ?air))
                  (over all (at ?turtlebot ?ground))
                  (at start (carrying ?drone ?box))
                  (at start (empty ?turtlebot))
                  (over all (airborne ?drone)))
  :effect (and (at end (not (carrying ?drone ?box)))
               (at end (empty ?drone))
               (at end (carrying ?turtlebot ?box))
               (at end (not (empty ?turtlebot))))
  )

(:durative-action hand-over-bot2drone
  :parameters (?drone - drone ?turtlebot - turtlebot ?box - box ?air - airwaypoint ?ground - waypoint)
  :duration (= ?duration 1)
  :condition (and (over all (over ?air ?ground))
                  (over all (at ?drone ?air))
                  (over all (at ?turtlebot ?ground))
                  (at start (carrying ?turtlebot ?box))
                  (at start (empty ?drone))
                  (over all (airborne ?drone)))
  :effect (and (at end (not (carrying ?turtlebot ?box)))
               (at end (empty ?turtlebot))
               (at end (carrying ?drone ?box))
               (at end (not (empty ?drone))))
  )

(:durative-action unload
  :parameters (?drone - drone ?box - box ?air - airwaypoint ?ground - waypoint ?person - person)
  :duration (= ?duration 1)
  :condition (and (over all (over ?air ?ground))
                  (at start (carrying ?drone ?box))
                  (over all (at ?drone ?air))
                  (over all (at ?person ?ground))
                  (over all (airborne ?drone)))
  :effect (and (at end (not (carrying ?drone ?box)))
               (at end (empty ?drone))
               (at end (handled ?person)))
  )

(:durative-action takeoff
  :parameters (?drone - drone ?air - airwaypoint ?ground - waypoint)
  :duration (= ?duration 1)
  :condition (and (over all (over ?air ?ground))
                  (at start (at ?drone ?ground)))
  :effect (and (at end (at ?drone ?air))
               (at end (not (at ?drone ?ground)))
               (at end (airborne ?drone)))
  )

(:durative-action land
  :parameters (?drone - drone ?air - airwaypoint ?ground - waypoint)
  :duration (= ?duration 1)
  :condition (and (over all (over ?air ?ground))
                  (at start (at ?drone ?air)))
  :effect (and (at end (at ?drone ?ground))
               (at end (not (at ?drone ?air)))
               (at end (not (airborne ?drone))))
  )
)


