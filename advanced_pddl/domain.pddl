(define (domain orbital_inspection)

    (:requirements
        :typing
        :strips
        :negative-preconditions
        :numeric-fluents
        :time
        :durative-actions
        :continuous-effects
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

        ; locking/unlocking movement
        (needs_inspection ?r - robot)

        ; sun
        (sun_present_between ?l1 ?l2 - location)
        (sun_present ?l - location)

        ; for the processes
        (transit ?r - robot ?l1 ?l2 - location)
        (inspecting ?r - robot ?c - components ?s - sensors)

        (daytime)
    )

    
    ;? here we store the resurces
    (:functions
        (orbit_time)

        ; fraction of max_charge_rate available given current sun exposure
        ; 0    - full shade, no charging
        ; 0.2  - grazing/transitional exposure (~30-90 degrees off-axis)
        ; 0.8  - direct sun exposure (within ~30 degrees)
        (sun_exposure ?r - robot)      

        ; battery
        (battery_level ?r - robot)
        (max_battery_level ?r - robot)
        (max_charge_rate ?r - robot)

        ; action costs
        (travel_time ?l1 ?l2 - location)
        (movement_cost ?r - robot)  ; the movement cost is constant as it is the robots motors
        (inspection_cost ?s - sensors)
        (inspection_time ?c - components)

        ; storage  
        (data_size ?c - components)
        (storage_used ?r - robot)
        (storage ?r - robot)
    )


    ;! The movement now becomes one. The process replaces the split
    (:durative-action move
        :parameters (
            ?r - robot 
            ?now ?go - location 
        )

        :duration (= ?duration (travel_time ?now ?go))

        :condition (and 
            (over all (not(robot_disabled ?r)))

            (at start (not (needs_inspection ?r)))
            (at start (robot_at ?r ?now))
            (at start (location_reachable ?now ?go))

            (at start (>= (battery_level ?r) (* (travel_time ?now ?go) (movement_cost ?r))))
        )
        :effect (and 
            ; update the spatial location and resources 
            (at start (not (robot_at ?r ?now)))
            
            (at start (transit ?r ?now ?go))
            
            ; we arrive to our new location
            (at end (robot_at ?r ?go))
            (at end (needs_inspection ?r))
            (at end (not (transit ?r ?now ?go)))
        )
    )

    ;! we inspect if the requirements of tools are met
    (:durative-action inspect
        :parameters (
            ?r - robot
            ?l - location
            ?c - components
            ?s - sensors
        )
        :duration (= ?duration (inspection_time ?c))
        :condition (and 
            (at start (needs_inspection ?r))

            (over all (not(robot_disabled ?r)))
            (over all (robot_at ?r ?l))
            (at start (component_at ?c ?l))

            (at start (has_sensor ?r ?s))
            (at start (requires_sensor ?c ?s))

            (at start (<= (+ (storage_used ?r) (data_size ?c)) (storage ?r)))
            (at start (>= (battery_level ?r) (inspection_cost ?s)))
        )
        :effect (and 
            ; we say that we are inspecting it at the start so we can drain and then we disable so the drainage stops
            (at start (inspecting ?r ?c ?s))
            (at end (not (inspecting ?r ?c ?s)))

            (at end (data_stored ?c))

            (at end (checked_component ?r ?c ?l))
            (at end (not (needs_inspection ?r)))
        )
    )
        
    
    ;! We specify a wait action which allows the robot to charge. But does nothing else
    (:durative-action wait
        :parameters ( 
            ?r - robot
            ?l - location
         )
        :duration (and (>= ?duration 0) (<= ?duration 200))
        :condition (and 
            (over all (robot_at ?r ?l))
            (over all  (not (needs_inspection ?r)))
            (over all (> (sun_exposure ?r) 0))
        )
        :effect (and)
    )
    

    ;! we just pass the location without inspecting it
    (:action skip_inspection
        :parameters (?r - robot)
        :precondition (needs_inspection ?r)
        :effect (and 
            (not (needs_inspection ?r))
        )
    )

        
    ;! the data is uploaded when we reach the docker
    (:action upload_data
        :parameters (
            ?r - robot 
            ?d - docking
            
        )
        :precondition (and
            (robot_at ?r ?d)
            (not(robot_disabled ?r))
        )
        :effect (and
            (assign (storage_used ?r) 0)
        )
    )

    ;! Process
    (:process charge
        :parameters (
            ?r - robot
        )
        :precondition (and
            (> (sun_exposure ?r) 0)
            (< (battery_level ?r) (max_battery_level ?r))
        )
        :effect (increase (battery_level ?r) 
            (* #t (* (sun_exposure ?r) (max_charge_rate ?r)))
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

    ; data collection and battery drainage for it
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
    
    
    

    ;! Events
    (:event battery_cap
        :parameters (
            ?r - robot
        )

        :precondition (> (battery_level ?r) (max_battery_level ?r))

        :effect (assign (battery_level ?r) (max_battery_level ?r))
    )

    (:event dead
        :parameters (?r - robot)

        :precondition (<= (battery_level ?r) 0)

        :effect (and
            (assign (battery_level ?r) 0)
            (robot_disabled ?r)
        )
    )
    

    ;! handling the day - night cycle
    (:event day_ends
        :parameters ()
        :precondition (and (daytime) (>= (orbit_time) 45))
        :effect (and
            (not (daytime))
            (assign (orbit_time) 0)
        )
    )

    (:event night_ends
        :parameters ()
        :precondition (and (not (daytime)) (>= (orbit_time) 45))
        :effect (and
            (daytime)
            (assign (orbit_time) 0)
        )
    )

    ; update to shade when night comes
    (:event update_exposure_night
        :parameters (?r - robot)
        :precondition (and (not (daytime)) (> (sun_exposure ?r) 0))
        :effect (assign (sun_exposure ?r) 0)
    )

    ; if the robot stationary and it is the peak of the sun
    (:event update_exposure_day_stationary_peak
        :parameters (?r - robot ?l - location)
        :precondition (and 
            (daytime) 
            (robot_at ?r ?l) 
            (sun_present ?l)
            (>= (orbit_time) 15) 
            (< (orbit_time) 30)
            (< (sun_exposure ?r) 0.75) 
        )
        :effect (assign (sun_exposure ?r) 0.8)
    )

    ; stationary and the sun is slanted
    (:event update_exposure_day_stationary_slanted
        :parameters (?r - robot ?l - location)
        :precondition (and 
            (daytime) 
            (robot_at ?r ?l) 
            (sun_present ?l)
            (or (< (orbit_time) 15) (>= (orbit_time) 30))
            (or (> (sun_exposure ?r) 0.25) (< (sun_exposure ?r) 0.15))
        )
        :effect (assign (sun_exposure ?r) 0.2)
    )

    ; if the robot is moving and the sun is at the peak
    (:event update_exposure_day_transit_full_peak
        :parameters (?r - robot ?now ?go - location)
        :precondition (and 
            (daytime) 
            (transit ?r ?now ?go) 
            (sun_present ?now) 
            (sun_present ?go) 
            (>= (orbit_time) 15) 
            (< (orbit_time) 30)
            (< (sun_exposure ?r) 0.75) ; we are making it 0.75 instead of 0.8 to avoid floating point precision errors
        )
        :effect (assign (sun_exposure ?r) 0.8)
    )

    ; moving but slanted
    (:event update_exposure_day_transit_full_slanted
        :parameters (?r - robot ?now ?go - location)
        :precondition (and 
            (daytime) 
            (transit ?r ?now ?go) 
            (sun_present ?now) 
            (sun_present ?go) 
            (or (< (orbit_time) 15) (>= (orbit_time) 30))
            (or (> (sun_exposure ?r) 0.25) (< (sun_exposure ?r) 0.15))
        )
        :effect (assign (sun_exposure ?r) 0.2)
    )

    ; if the robot is moving from/to a shaded place to a light one
    (:event update_exposure_day_partial
        :parameters (?r - robot ?now ?go - location)
        :precondition (and 
            (daytime) 
            (transit ?r ?now ?go)
            (or (and (sun_present ?now) (not (sun_present ?go)))
                (and (not (sun_present ?now)) (sun_present ?go)))
            (or (> (sun_exposure ?r) 0.25) (< (sun_exposure ?r) 0.15)) 
        )
        :effect (assign (sun_exposure ?r) 0.2)
    )

    ; there is day but the robot is in a perpetual shaded part of the map
    (:event update_exposure_day_shade
        :parameters (?r - robot ?l - location)
        :precondition (and 
            (daytime) 
            (robot_at ?r ?l) 
            (not (sun_present ?l))
            (> (sun_exposure ?r) 0)
        )
        :effect (assign (sun_exposure ?r) 0)
    )

    ; going between two shaded places
    (:event update_exposure_day_transit_shade
        :parameters (?r - robot ?now ?go - location)
        :precondition (and 
            (daytime) 
            (transit ?r ?now ?go) 
            (not (sun_present ?now)) 
            (not (sun_present ?go)) 
            (> (sun_exposure ?r) 0)
        )
        :effect (assign (sun_exposure ?r) 0)
    )
)