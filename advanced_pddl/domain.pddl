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
            (at start (not (needs_inspection ?r)))
            (at start (robot_at ?r ?now))
            (at start (location_reachable ?now ?go))
            (at start (>= (battery_level ?r) (movement_cost ?r)))
            (over all (not(robot_disabled ?r)))
        )
        :effect (and 
            ; update the spatial location and resources 
            (at start (not (robot_at ?r ?now)))

            (at start (decrease (battery_level ?r) (* (travel_time ?now ?go)(movement_cost ?r))))


            ; Depending where we are going we update the sun level so we can have an idea for the charging process

            ; when the movement is fully under the sun
            (at start 
                (when (and (daytime) (sun_present ?now) (sun_present ?go))
                    (assign (sun_exposure ?r) 0.8)
                )
            ) 

            ; when we go or end up from sun to shade
            (at start 
                (when (and (daytime) (or 
                        (and (sun_present ?now) (not(sun_present ?go)))
                        (and (not(sun_present ?now)) (sun_present ?go))
                      )
                    (assign (sun_exposure ?r) 0.2))
                )
            )

            ; fully in the shade
            (at start 
                (when (and (not (sun_present ?now)) (not (sun_present ?go)))
                    (assign (sun_exposure ?r) 0)
                )
            )

            
            ; we arrive to our new location
            (at end (robot_at ?r ?go))
            (at end (needs_inspection ?r))

            (at end (when (and (daytime) (sun_present ?go))
                (assign (sun_exposure ?r) 2)))
            (at end (when (or (not (daytime)) (not (sun_present ?go)))
                (assign (sun_exposure ?r) 0)))
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
            (at start (decrease (battery_level ?r) (inspection_cost ?s)))

            (at end (increase (storage_used ?r) (data_size ?c)))
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
        :effect (
            (increase (orbit_time) (* #t 1))
        )
    )
    

    ;! Events
    (:event battery_cap
        :parameters (
            ?r - robot
        )
        :precondition (
            (>= (battery_level ?r) (max_battery_level ?r))
        )
        :effect (
            (assign (battery_level ?r) (max_battery_level ?r))
        )
    )

    (:event dead
        :parameters (?r - robot)

        :precondition (<= (battery_level ?r) 0)

        :effect (and
            (assign (battery_level ?r) 0)
            (robot_disabled ?r)
        )
    )
    
    (:event day_ends
        :parameters (
            ?r - robot
            ?l - location
        )
        :precondition (and
            (daytime)
            (>= (orbit_time) 45)
            (robot_at ?r ?l)
        )
        :effect (and
            (not(daytime))
            (assign (sun_exposure ?r) 0)        ; the robot has no light so it is night everywhere
            (assign (orbit_time) 0)
        )
    )


    (:event night_ends
        :parameters (
            ?r - robot
            ?l - location
        )
        :precondition (and
            (not (daytime))
            (>= (orbit_time) 45)
            (robot_at ?r ?l)
        )
        :effect (and
            (daytime)
            (assign (orbit_time) 0)

            (when (sun_present ?l) 
                (assign (sun_exposure ?r) 2)
            )

            (when (not (sun_present ?l)) 
                (assign (sun_exposure ?r) 0)
            )
        )
    )
)