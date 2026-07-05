;; Problem 2: star topology through a hub, three
;; components each needing a different sensor modality, and the
;; location of the radator is always in shade
;;
;; The limited resources
;;
;; 1. STORAGE: cap is 14. data_size(panel)=5, data_size(radiator)=8,
;;    data_size(antenna)=10. All three together = 23 > 14, and no
;;    combination except panel+radiator (=13) fits under the cap
;;    together - antenna must always be uploaded separately. So 
;;    the planner must decide which to inspect together and which alone
;;
;; 2. BATTERY: the robot starts too low (19) to complete the whole
;;    mission without recharging, and radiator-site has no sun, so
;;    charging is only possible before/after that branch, not
;;    during it. The robot must run itself down to the <=5 trigger
;;    at a sunlit hub, run a full charge_start -> charge* ->
;;    stop_charging cycle, and only then dive into the shadowed
;;    radiator branch with enough reserve to get back out.


(define (problem hard_problem)
  (:domain orbital_inspection)

  (:objects
    rob1 - robot
    dock - docking
    hub panel_site radiator_site antenna_site - location
    panel_1 radiator_1 antenna_1 - components
    camera thermal_camera highres_camera - sensors
  )

  (:init
    (robot_at rob1 dock)

    (location_reachable dock hub) (location_reachable hub dock)
    (location_reachable hub panel_site) (location_reachable panel_site hub)
    (location_reachable hub radiator_site) (location_reachable radiator_site hub)
    (location_reachable hub antenna_site) (location_reachable antenna_site hub)

    ;; radiator_site is a permanently shadowed maintenance bay:
    ;; no (sun_present radiator_site) and no sun on the edges to/from it
    (sun_present dock)
    (sun_present hub)
    (sun_present panel_site)
    (sun_present antenna_site)

    (sun_present_between dock hub) (sun_present_between hub dock)
    (sun_present_between hub panel_site) (sun_present_between panel_site hub)
    (sun_present_between hub antenna_site) (sun_present_between antenna_site hub)

    (component_at panel_1 panel_site)
    (requires_sensor panel_1 camera)

    (component_at radiator_1 radiator_site)
    (requires_sensor radiator_1 thermal_camera)

    (component_at antenna_1 antenna_site)
    (requires_sensor antenna_1 highres_camera)

    (has_sensor rob1 camera)
    (has_sensor rob1 thermal_camera)
    (has_sensor rob1 highres_camera)

    (= (battery_level rob1) 19)
    (= (storage_used rob1) 0)
    (= (storage rob1) 14)

    (= (cost dock hub) 10) (= (cost hub dock) 10)
    (= (cost hub panel_site) 10) (= (cost panel_site hub) 10)
    (= (cost hub radiator_site) 8) (= (cost radiator_site hub) 8)
    (= (cost hub antenna_site) 10) (= (cost antenna_site hub) 10)

    (= (inspection_cost camera) 5)
    (= (inspection_cost thermal_camera) 10)
    (= (inspection_cost highres_camera) 10)

    (= (data_size panel_1) 5)
    (= (data_size radiator_1) 8)
    (= (data_size antenna_1) 10)
  )

  (:goal (and
    (checked_component rob1 panel_1 panel_site)
    (checked_component rob1 radiator_1 radiator_site)
    (checked_component rob1 antenna_1 antenna_site)
    (robot_at rob1 dock)
    (= (storage_used rob1) 0)
  ))
)
