(define (problem p_rescue)
(:domain d_rescue)
(:objects d0 - DRONE depo l0 l1 - LOC p0 p1 - PERSON b0 b1 b2 - BOX med food - BTYPE)
(:init (at b0 depo) (at b1 depo) (at b2 depo) (at d0 depo) (at p0 l0) (at p1 l1)
       (contains b0 med) (contains b1 med) (contains b2 food)
       (empty d0)
       (= (total-cost) 0) (= (fly-cost depo depo) 0) (= (fly-cost l0 l0) 0) (= (fly-cost l1 l1) 0)
       (= (fly-cost depo l0) 100) (= (fly-cost l0 depo) 100)
       (= (fly-cost depo l1) 100) (= (fly-cost l1 depo) 100)
       (= (fly-cost l0 l1) 10) (= (fly-cost l1 l0) 10)
)
(:goal (and (has p0 med) (has p1 med) (has p0 food)))
(:metric minimize (total-cost))
)
