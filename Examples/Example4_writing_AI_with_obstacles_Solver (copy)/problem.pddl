(define (problem draw_ai_problem)
  (:domain draw_ai)

  (:objects
    turtle - turtle
    a_top a_left_leg a_right_leg a_middle_left a_middle_right i_top i_bottom unknown - location
  )

  (:init
    (at turtle unknown) 
    (pen-enabled turtle)
    (permissible-marking a_top a_left_leg)
    (permissible-marking a_top a_right_leg)
    (permissible-marking a_middle_left a_middle_right)
    (permissible-marking i_top i_bottom)
    (permissible-marking a_left_leg a_top)
    (permissible-marking a_right_leg a_top)
    (permissible-marking a_middle_right a_middle_left)
    (permissible-marking i_bottom i_top)
  )

  (:goal
    (and
      (marked a_top a_left_leg) 
      (marked a_top a_right_leg)  
      (marked a_middle_left a_middle_right)
      (marked i_top i_bottom)
    )
  )

  (:metric minimize (total-cost))
)

