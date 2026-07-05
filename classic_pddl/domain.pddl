(define (domain orbital_inspection)

    (:requirements
        :typing
        :strips
        :negative-preconditions
        :numeric-fluents
    )

    (:types
        robot - object

        location - object
        docking - location

        components - object
        sensors - object
    )

    (:predicates
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
        (charging ?r)

        ; sun
        (sun_present_between ?l1 ?l2 - location)
        (sun_present ?l - location)
        

    )

    
    ;? here we store the resurces
    (:functions
        ; battery
        (battery_level ?r - robot)
        (cost ?l1 ?l2 - location)
        (inspection_cost ?s - sensors)
        

        ; storage  
        (data_size ?c - components)
        (storage_used ?r - robot)
        (storage ?r - robot)
    )


    ;! Movement separated to be with the sun or without the sun
    (:action move_sun
        :parameters (
            ?r - robot 
            ?now ?go - location 
        )
        
        :precondition (and 
            (not (needs_inspection ?r))
            (not (charging ?r))
            (robot_at ?r ?now)
            (location_reachable ?now ?go)
            (sun_present_between ?now ?go)
            (>= (battery_level ?r) (/ (* (cost ?now ?go) 6) 10))
        )
    
        :effect (and 
            (not (robot_at ?r ?now))
            (robot_at ?r ?go)
            ; we supose that while moving the sun compesates 40% of the power
            (decrease (battery_level ?r) (/ (* (cost ?now ?go) 6) 10))

            ;robot must evaluate/inspect the new node before moving again
            (needs_inspection ?r)
        )
    )

    (:action move_without_sun
        :parameters (
            ?r - robot 
            ?now ?go - location 
        )
        :precondition (and 
            (not (needs_inspection ?r))
            (not (charging ?r))
            (robot_at ?r ?now)
            (location_reachable ?now ?go)
            (not (sun_present_between ?now ?go))
            (>= (battery_level ?r) (cost ?now ?go))
        )
        :effect (and 
            (not (robot_at ?r ?now))
            (robot_at ?r ?go)
            ; we supose that while moving the sun compesates 40% of the power
            (decrease (battery_level ?r) (cost ?now ?go))

            ;robot must evaluate/inspect the new node before moving again
            (needs_inspection ?r)
        )
    )

    ;! we just pass the location without inspecting it
    (:action skip_inspection
        :parameters (?r - robot)
        :precondition (needs_inspection ?r)
        :effect (and 
            (not (needs_inspection ?r))
        )
    )


    ;! we inspect if the requirements of tools are met
    (:action inspect_sun
        :parameters (
            ?r - robot
            ?l - location
            ?c - components
            ?s - sensors
        )
        :precondition (and 
            (needs_inspection ?r)
            (sun_present ?l)
            (not (charging ?r))

            (robot_at ?r ?l)
            (component_at ?c ?l)

            (has_sensor ?r ?s)
            (requires_sensor ?c ?s)

            (<= (+ (storage_used ?r) (data_size ?c)) (storage ?r))
            (>= (battery_level ?r) (/ (* (inspection_cost ?s) 2) 10))
        )
        :effect (and 
            (increase (storage_used ?r) (data_size ?c))
            (data_stored ?c)

            ; i chose to reduce by 20, since we are not moving and inspecting is just using the sensors not using the motors of the wheels
            (decrease (battery_level ?r) (/ (* (inspection_cost ?s) 2) 10))

            (checked_component ?r ?c ?l)
            (not (needs_inspection ?r)) ; now we can move
        )
    )

    (:action inspect_without_sun
        :parameters (
            ?r - robot
            ?l - location
            ?c - components
            ?s - sensors
        )
        :precondition (and 
            (needs_inspection ?r)
            (not (sun_present ?l))
            (not (charging ?r))

            (robot_at ?r ?l)
            (component_at ?c ?l)

            (has_sensor ?r ?s)
            (requires_sensor ?c ?s)

            (<= (+ (storage_used ?r) (data_size ?c)) (storage ?r))
            (>= (battery_level ?r) (inspection_cost ?s))
        )
        :effect (and 
            (increase (storage_used ?r) (data_size ?c))
            (data_stored ?c)

            (decrease (battery_level ?r) (inspection_cost ?s))

            (checked_component ?r ?c ?l)
            (not (needs_inspection ?r)) ; now we can move
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
        )
        :effect (and
            (assign (storage_used ?r) 0)
        )
    )

    ;! we charge if the battery is low
    (:action charge_start
        :parameters (
            ?r - robot 
            ?l - location
        )
        ; if the robot has less than 10% and there is the sun it starts to charge on its own
        :precondition (and
            (robot_at ?r ?l)
            (<= (battery_level ?r) 5)
            (sun_present ?l)
            (not (charging ?r))
        )
        :effect (and
            (charging ?r)
        )
    )  

    ; this can loop until max since the ISS has 45 mins sun and 45 night
    (:action charge
        :parameters (
            ?r - robot 
            ?l - location
        )
        :precondition (and 
            (robot_at ?r ?l)
            (charging ?r)
            (sun_present ?l)
            (<= (+ (battery_level ?r) 5) 90) ; we check if the next result is over the cap
        )
        :effect (and 
            (increase (battery_level ?r) 5)
        )
    )

    ; the plannar can stop charging when its sees fit
    (:action stop_charging
        :parameters (?r - robot)
        :precondition (and 
            (charging ?r)
        )
        :effect (and 
            (not (charging ?r))
        )
    )


)