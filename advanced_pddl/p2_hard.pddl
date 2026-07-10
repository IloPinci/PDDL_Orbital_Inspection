(define (problem p2_hard)
    (:domain orbital_inspection)

    (:objects
        r1 - robot
        dock - docking
        loc_a - location
        loc_b - location
        panel1 - components
        panel2 - components
        cam - sensors
        spectro - sensors
    )

    (:init
        ;; spatial layout: loc_b is only reachable via loc_a
        (robot_at r1 dock)
        (location_reachable dock loc_a)
        (location_reachable loc_a dock)
        (location_reachable loc_a loc_b)
        (location_reachable loc_b loc_a)
        (component_at panel1 loc_a)
        (component_at panel2 loc_b)

        ;; sensing: two different sensors, both onboard
        (requires_sensor panel1 cam)
        (requires_sensor panel2 spectro)
        (has_sensor r1 cam)
        (has_sensor r1 spectro)

        ;; lighting: dock and loc_a are lit; loc_b is deliberately in shadow
        (sun_present dock)
        (sun_present loc_a)
        ;; (no `sun_present loc_b` fact -- it's shaded)
        (daytime)

        ;; orbital clock: deploy right at sunrise, start of orbit_index 0
        (= (orbit_time) 0)
        (= (orbit_index) 0)
        (= (solar_factor) 0)

        ;; robot state
        (not (robot_disabled r1))
        (not (uploading r1))
        (= (sun_exposure r1) 2)
        (= (activity_timer r1) 0)
        (= (inspection_timer r1) 0)

        ;; battery: has real margin only because of concurrent charging
        (= (battery_level r1) 18)
        (= (max_battery_level r1) 30)
        (= (max_charge_rate r1) 6)
        (= (movement_cost r1) 1)

        ;; travel: dock <-> loc_a <-> loc_b, no dock <-> loc_b shortcut
        (= (travel_time dock loc_a) 4)
        (= (travel_time loc_a dock) 4)
        (= (travel_time loc_a loc_b) 3)
        (= (travel_time loc_b loc_a) 3)
        ;; placeholder for the unreachable pair, so grounding never
        ;; chokes on an undefined fluent (location_reachable already
        ;; blocks these actions from ever firing)
        (= (travel_time dock loc_b) 999)
        (= (travel_time loc_b dock) 999)

        ;; inspection
        (= (inspection_cost cam) 6)
        (= (inspection_time panel1) 4)
        (= (inspection_cost spectro) 9)
        (= (inspection_time panel2) 5)

        ;; data & storage -- both fit at once, no mid-mission dump needed
        (= (data_size panel1) 15)
        (= (data_size panel2) 20)
        (= (storage r1) 40)
        (= (storage_used r1) 0)

        ;; upload: rate is high enough to fully drain within a short
        ;; window, but only if start_upload is called promptly on arrival
        (= (upload_cost r1) 1)
        (= (upload_rate r1) 20)
    )

    (:goal (and
        (checked_component r1 panel1 loc_a)
        (checked_component r1 panel2 loc_b)
        (data_stored panel1)
        (data_stored panel2)
        (robot_at r1 dock)
        (= (storage_used r1) 0)
        (not (uploading r1))
    ))
)