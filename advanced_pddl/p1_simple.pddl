(define (problem p1_simple)
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
        ;; simple layout: both locations directly reachable from dock,
        ;; and from each other -- no forced relay this time
        (robot_at r1 dock)
        (location_reachable dock loc_a)
        (location_reachable loc_a dock)
        (location_reachable dock loc_b)
        (location_reachable loc_b dock)
        (location_reachable loc_a loc_b)
        (location_reachable loc_b loc_a)
        (component_at panel1 loc_a)
        (component_at panel2 loc_b)

        ;; sensing: same two modalities, both onboard
        (requires_sensor panel1 cam)
        (requires_sensor panel2 spectro)
        (has_sensor r1 cam)
        (has_sensor r1 spectro)

        ;; lighting: everywhere is lit -- no shadow penalty at all
        (sun_present dock)
        (sun_present loc_a)
        (sun_present loc_b)
        (daytime)

        ;; orbital clock: deploy right at solar peak, plenty of orbit
        ;; left before day_ends, and comm window (orbit_time 15-25,
        ;; orbit_index <= 2) is comfortably reachable from any location
        (= (orbit_time) 10)
        (= (orbit_index) 0)
        (= (solar_factor) 1.0)

        ;; robot state
        (not (robot_disabled r1))
        (not (uploading r1))
        (= (sun_exposure r1) 2)
        (= (activity_timer r1) 0)
        (= (inspection_timer r1) 0)

        ;; battery: generous margin, and the robot is charging the
        ;; whole time since it's never in shade
        (= (battery_level r1) 25)
        (= (max_battery_level r1) 30)
        (= (max_charge_rate r1) 6)
        (= (movement_cost r1) 1)

        ;; travel: short hops, everything close to dock
        (= (travel_time dock loc_a) 2)
        (= (travel_time loc_a dock) 2)
        (= (travel_time dock loc_b) 2)
        (= (travel_time loc_b dock) 2)
        (= (travel_time loc_a loc_b) 2)
        (= (travel_time loc_b loc_a) 2)

        ;; inspection: quick and cheap
        (= (inspection_cost cam) 3)
        (= (inspection_time panel1) 3)
        (= (inspection_cost spectro) 4)
        (= (inspection_time panel2) 3)

        ;; data & storage: ample headroom, no mid-mission dump needed
        (= (data_size panel1) 10)
        (= (data_size panel2) 10)
        (= (storage r1) 50)
        (= (storage_used r1) 0)

        ;; upload: fast, and the comm window is wide enough that the
        ;; robot doesn't need to rush back to catch it
        (= (upload_cost r1) 1)
        (= (upload_rate r1) 15)
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