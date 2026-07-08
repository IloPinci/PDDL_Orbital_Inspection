(define (domain orbital_inspection)

    (:requirements
        :typing
        :strips
        :negative-preconditions
        :numeric-fluents
        :time
        :continuous-effects
        :disjunctive-preconditions
    )

    (:types
        robot - object

        location - object
        docking - location

        components - object
        sensors - object
    )

    (:predicates
        (robot_disabled ?r - robot)
        (robot_at ?r - robot ?l - location)
        (component_at ?c - components ?l - location)
        (location_reachable ?l1 ?l2 - location)
        
        ; which sensor is required for which task
        (requires_sensor ?c - components ?s - sensors)
        ; if the robot has the hardware to do the task
        (has_sensor ?r - robot ?s - sensors)

        ; track the state of the mission
        (checked_component ?r - robot ?c - components ?l - location)
        (data_stored ?c - components)
        (uploading ?r - robot)

        ; locking/unlocking movement
        (needs_inspection ?r - robot)

        ; sun
        (sun_present ?l - location)

        ; for the processes
        (transit ?r - robot ?l1 ?l2 - location)
        (inspecting ?r - robot ?c - components ?s - sensors)

        (daytime)
        (transmit_window_open)
    )

    
    ;? here we store the resurces
    (:functions 
        (orbit_time)
        (orbit_index)
        (solar_factor)

        ; fraction of max_charge_rate available given current sun exposure
        ; 0 - full shade, no charging
        ; 1 - going from light to shaded and vice versa. So 50% of the road in shade and 50 in light
        ; 2  - direct sun exposure 
        (sun_exposure ?r - robot)      

        ; battery
        (battery_level ?r - robot)
        (max_battery_level ?r - robot)
        (max_charge_rate ?r - robot)

        ; synchronizing timers
        (travel_time ?l1 ?l2 - location)
        (inspection_time ?c - components)
        (activity_timer ?r - robot)

        ; action costs
        (movement_cost ?r - robot)  ; the movement cost is constant as it is the robots motors
    (inspection_cost ?s - sensors)
        (upload_cost ?r - robot)

        ; storage  
        (upload_rate ?r - robot)
        (data_size ?c - components)
        (storage_used ?r - robot)
        (storage ?r - robot)
        
    )


    ;! The movement now becomes one. The process replaces the split
    (:action move_start
        :parameters (?r - robot ?now ?go - location)
        :precondition (and
            (not (robot_disabled ?r))
            (not (needs_inspection ?r))
            (robot_at ?r ?now)
            (location_reachable ?now ?go)
            (>= (battery_level ?r) (* (travel_time ?now ?go) (movement_cost ?r)))
        )
        :effect (and
            (not (robot_at ?r ?now))
            (transit ?r ?now ?go)
            (assign (activity_timer ?r) 0)
        )
    )

    ;! we inspect if the requirements of tools are met
    (:action inspect_start
        :parameters (?r - robot ?l - location ?c - components ?s - sensors)
        :precondition (and
            (needs_inspection ?r)
            (not (robot_disabled ?r))
            (robot_at ?r ?l)
            (component_at ?c ?l)
            (has_sensor ?r ?s)
            (requires_sensor ?c ?s)
            (<= (+ (storage_used ?r) (data_size ?c)) (storage ?r))
            (>= (battery_level ?r) (inspection_cost ?s))
        )
        :effect (and
            (inspecting ?r ?c ?s)
            (assign (activity_timer ?r) 0)
        )
    )
    

    ;! we just pass the location without inspecting it
    (:action skip_inspection
        :parameters (
            ?r - robot
            ?s - sensors
            ?c - components
        )
        :precondition (and
            (needs_inspection ?r)
            (not (inspecting ?r ?c ?s))
        )
        :effect (and 
            (not (needs_inspection ?r))
        )
    )


    ;! the data is uploaded when we reach the docker
    (:action start_upload
        :parameters (?r - robot ?d - docking)
        :precondition (and 
            (robot_at ?r ?d)
            (not (uploading ?r))
            (> (storage_used ?r) 0)
            (not (robot_disabled ?r))
        )
        :effect (uploading ?r)
    )

    (:action stop_upload
        :parameters (?r - robot)
        :precondition (uploading ?r)
        :effect (not (uploading ?r))
    )

    
    ;! Process

    ;? clock for continuous processes
    (:process move_clock
        :parameters (?r - robot ?now ?go - location)
        :precondition (transit ?r ?now ?go)
        :effect (increase (activity_timer ?r) (* #t 1))
    )

    (:process inspect_clock
        :parameters (?r - robot ?c - components ?s - sensors)
        :precondition (and
            (inspecting ?r ?c ?s)
            (not (robot_disabled ?r))
        )
        :effect (increase (activity_timer ?r) (* #t 1))
    )

    
    (:process charge
        :parameters (
            ?r - robot
        )
        :precondition (and
            (> (sun_exposure ?r) 0)
            (< (battery_level ?r) (max_battery_level ?r))
        )
        :effect (increase (battery_level ?r) 
            (* #t (* (/ (sun_exposure ?r) 2) (* (solar_factor) (max_charge_rate ?r))))
        )
    )

    (:process orbit_tick
        :parameters ()
        :precondition (>= (orbit_time) 0)       ; this is always true so the update is always
        :effect (increase (orbit_time) (* #t 1))
    )

    ; we drain battery drainage for movement
    (:process drain_battery
        :parameters (
            ?r - robot
            ?now ?go - location
        )
        :precondition (and 
            (transit ?r ?now ?go)
            (not (robot_disabled ?r))
        )
        :effect (and
            (decrease (battery_level ?r) (* #t (movement_cost ?r)))
        )
    )

    ; data collection / transmission and battery drainage for it
    (:process collect_data
        :parameters (
            ?r - robot
            ?c - components
            ?s - sensors
        )
        :precondition (and
            (inspecting ?r ?c ?s)
            (not (robot_disabled ?r))
        )
        :effect (and
            (decrease (battery_level ?r) 
                (* #t (/ (inspection_cost ?s) (inspection_time ?c)))
            )

            (increase (storage_used ?r) 
                (* #t (/ (data_size ?c) (inspection_time ?c)))
            )
        )
    )

    (:process transmit_data
        :parameters (
            ?r - robot
            ?d - docking
        )
        :precondition (and 
            (uploading ?r)
            (not (robot_disabled ?r))
            (transmit_window_open)
            (robot_at ?r ?d)
        )
        :effect (and
            (decrease (storage_used ?r) (* #t (upload_rate ?r)))
            (decrease (battery_level ?r) (* #t (upload_cost ?r)))
        )
    )

    ; sunrise
    (:process solar_intensity_rise
        :parameters ()
        :precondition (and 
            (daytime) 
            (< (orbit_time) 22.5)   ; bc the iss has 45 mins of light
        )
        :effect (increase (solar_factor) (* #t (/ 1 22.5))) 
    )

    ; sunset
    (:process solar_intensity_fall
        :parameters ()
        :precondition (and 
            (daytime) 
            (>= (orbit_time) 22.5)
        )
        :effect (decrease (solar_factor) (* #t (/ 1 22.5)))
    )
    
    
    

    ;! Events
    (:event dead
        :parameters (?r - robot)

        :precondition (<= (battery_level ?r) 0)

        :effect (and
            (assign (battery_level ?r) 0)
            (robot_disabled ?r)
        )
    )

    (:event move_end
        :parameters (?r - robot ?now ?go - location)
        :precondition (and
            (transit ?r ?now ?go)
            (>= (activity_timer ?r) (travel_time ?now ?go))
        )
        :effect (and
            (not (transit ?r ?now ?go))
            (robot_at ?r ?go)
            (needs_inspection ?r)
        )
    )
    
    (:event inspect_end
        :parameters (?r - robot ?l - location ?c - components ?s - sensors)
        :precondition (and
            (inspecting ?r ?c ?s)
            (>= (activity_timer ?r) (inspection_time ?c))
        )
        :effect (and
            (not (inspecting ?r ?c ?s))
            (data_stored ?c)
            (checked_component ?r ?c ?l)
            (not (needs_inspection ?r))
        )
    )

    ;! global change of time 
    (:event day_ends
        :parameters ()
        :precondition (and (daytime) (>= (orbit_time) 45))
        :effect (and
            (not (daytime))
            (assign (orbit_time) 0)
            (assign (solar_factor) 0)
        )
    )

    (:event night_ends
        :parameters ()
        :precondition (and (not (daytime)) (>= (orbit_time) 45))
        :effect (and
            (daytime)
            (assign (orbit_time) 0)
            (assign (solar_factor) 0)
            (increase (orbit_index) 1)
        )
    )

    ; 90 mins per full orbit. 22.5 degree longitufe shift per orbit. In one day we have 16 orbits
    (:event wrap_orbit_idx
        :parameters ()
        :precondition (>= (orbit_index) 16)
        :effect (assign (orbit_index) 0)    ; new day
    )
    


    ;! Light situation for the robot
    ;? during the night we have shade everywhere
    (:event update_exposure_night
        :parameters (?r - robot)
        :precondition (and (not (daytime)) (not (= (sun_exposure ?r) 0)))
        :effect (assign (sun_exposure ?r) 0)
    )

    ;? if the robot stationary and it is in the shade
    (:event update_exposure_day_shade
        :parameters (?r - robot ?l - location)
        :precondition (and 
            (daytime) 
            (robot_at ?r ?l) 
            (not (sun_present ?l))
            (not (= (sun_exposure ?r) 0))
        )
        :effect (assign (sun_exposure ?r) 0)
    )

    ;? stationary and there is sun
    (:event update_exposure_day_full_sun
        :parameters (?r - robot ?l - location)
        :precondition (and 
            (daytime) 
            (robot_at ?r ?l) 
            (sun_present ?l)
            (not (= (sun_exposure ?r) 2)) 
        )
        :effect (assign (sun_exposure ?r) 2)
    )

    ;? if the robot is moving from shade to shade
    (:event update_exposure_day_transit_shade
        :parameters (?r - robot ?now ?go - location)
        :precondition (and 
            (daytime) 
            (transit ?r ?now ?go) 
            (not (sun_present ?now)) 
            (not (sun_present ?go)) 
            (not (= (sun_exposure ?r) 0))
        )
        :effect (assign (sun_exposure ?r) 0)
    )

    ;? moving from light to light
    (:event update_exposure_day_transit_full
        :parameters (?r - robot ?now ?go - location)
        :precondition (and 
            (daytime) 
            (transit ?r ?now ?go) 
            (sun_present ?now) 
            (sun_present ?go) 
            (not (= (sun_exposure ?r) 2))
        )
        :effect (assign (sun_exposure ?r) 2)
    )

    ;? if the robot is moving from/to a shaded place to a light one
    (:event update_exposure_day_transit_partial
        :parameters (?r - robot ?now ?go - location)
        :precondition (and 
            (daytime) 
            (transit ?r ?now ?go)
            (or (and (sun_present ?now) (not (sun_present ?go)))
                (and (not (sun_present ?now)) (sun_present ?go)))
            (not (= (sun_exposure ?r) 1)) 
        )
        :effect (assign (sun_exposure ?r) 1)
    )


    ;! transmision

    ;? usa has spans 60 degrees. So if we assume that we start immediately above it we will continue to be in it for 3 orbits
    (:event comm_window_starts
        :parameters ()
        :precondition (and 
            (daytime)
            (not (transmit_window_open)) 
            (<= (orbit_index) 2)             
            (>= (orbit_time) 15)     ; we assume that the iss is above USA for 10 mins
            (< (orbit_time) 25)
        )
        :effect (transmit_window_open)
    )

    (:event comm_window_ends
        :parameters ()
        :precondition (and 
            (daytime)
            (transmit_window_open) 
            (or 
                (>= (orbit_time) 25)         
                (> (orbit_index) 2)          ; just to make sure
            )
        )
        :effect (not (transmit_window_open))
    )

    (:event upload_complete
        :parameters (?r - robot)
        :precondition (and 
            (uploading ?r) 
            (<= (storage_used ?r) 0)
        )
        :effect (and
            (not (uploading ?r))
            (assign (storage_used ?r) 0)
        )
    )

)