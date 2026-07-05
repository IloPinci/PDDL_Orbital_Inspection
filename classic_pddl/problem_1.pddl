(define (problem simple_problem)
  (:domain orbital_inspection)

  (:objects
    rob1 - robot
    dock - docking
    site_a - location
    panel_1 - components
    camera thermal_camera highres_camera - sensors
  )

  (:init
    (robot_at rob1 dock)

    (location_reachable dock site_a)
    (location_reachable site_a dock)

    (sun_present dock)
    (sun_present site_a)
    (sun_present_between dock site_a)
    (sun_present_between site_a dock)

    (component_at panel_1 site_a)
    (requires_sensor panel_1 camera)

    (has_sensor rob1 camera)
    (has_sensor rob1 thermal_camera)
    (has_sensor rob1 highres_camera)

    (= (battery_level rob1) 50)
    (= (storage_used rob1) 0)
    (= (storage rob1) 20)

    (= (cost dock site_a) 10)
    (= (cost site_a dock) 10)

    (= (inspection_cost camera) 5)
    (= (inspection_cost thermal_camera) 10)
    (= (inspection_cost highres_camera) 10)

    (= (data_size panel_1) 5)
  )

  (:goal (and
    (checked_component rob1 panel_1 site_a)
    (robot_at rob1 dock)
    (= (storage_used rob1) 0)
  ))
)