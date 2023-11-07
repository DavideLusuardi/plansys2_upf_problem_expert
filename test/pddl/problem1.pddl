(define (problem plansys2_1)
  (:domain simple)
  (:objects
    Leia - robot
    entrance - room
    kitchen - room
    bedroom - room
    dinning - room
    bathroom - room
    chargingroom - room
  )
  (:init
    (connected entrance dinning)
    (connected dinning entrance)
    (connected dinning kitchen)
    (connected kitchen dinning)
    (connected dinning bedroom)
    (connected bedroom dinning)
    (connected bathroom bedroom)
    (connected bedroom bathroom)
    (connected chargingroom kitchen)
    (connected kitchen chargingroom)
    (charging_point_at chargingroom)
    (battery_low leia)
    (robot_at leia entrance)
    (= (distance_travelled) 0)
  )

  ;; The goal is to have both packages delivered to their destinations:
  (:goal
    (and(robot_at leia bathroom))
  )
)
