(define (problem orbital_inspection_p1)
    (:domain orbital_inspection)

    (:objects
        dock - docking
        p1 p2 p3 - location
        robot1 - robot
        solar_panel antenna radiator - components
        cam thermal hires - sensors
    )

    (:init
        (robot_at robot1 dock)

        ; strict cycle, no shortcuts -> forces a fixed visiting order
        (location_reachable dock p1) (location_reachable p1 dock)
        (location_reachable p1 p2)   (location_reachable p2 p1)
        (location_reachable p2 p3)   (location_reachable p3 p2)
        (location_reachable p3 dock) (location_reachable dock p3)

        (component_at solar_panel p1)
        (component_at antenna p2)
        (component_at radiator p3)

        (requires_sensor solar_panel cam)
        (requires_sensor antenna hires)
        (requires_sensor radiator thermal)

        (has_sensor robot1 cam)
        (has_sensor robot1 thermal)
        (has_sensor robot1 hires)

        ; sun geometry: p2 is in the platform's own shadow
        (sun_present dock)
        (sun_present p1)
        (sun_present p3)

        (daytime)
        (= (orbit_time) 0)
        (= (sun_exposure robot1) 0)

        (= (battery_level robot1) 80)
        (= (max_battery_level robot1) 100)
        (= (max_charge_rate robot1) 1)

        (= (travel_time dock p1) 5) (= (travel_time p1 dock) 5)
        (= (travel_time p1 p2) 5)   (= (travel_time p2 p1) 5)
        (= (travel_time p2 p3) 5)   (= (travel_time p3 p2) 5)
        (= (travel_time p3 dock) 5) (= (travel_time dock p3) 5)

        (= (movement_cost robot1) 2)

        (= (inspection_cost cam) 5)
        (= (inspection_cost thermal) 10)
        (= (inspection_cost hires) 15)

        (= (inspection_time solar_panel) 10)
        (= (inspection_time antenna) 10)
        (= (inspection_time radiator) 10)

        (= (data_size solar_panel) 5)
        (= (data_size antenna) 8)
        (= (data_size radiator) 10)

        (= (storage_used robot1) 0)
        (= (storage robot1) 30)
    )

    (:goal (and
        (checked_component robot1 solar_panel p1)
        (checked_component robot1 antenna p2)
        (checked_component robot1 radiator p3)
        (data_stored solar_panel)
        (data_stored antenna)
        (data_stored radiator)
        (robot_at robot1 dock)
        (= (storage_used robot1) 0)
    ))
)
