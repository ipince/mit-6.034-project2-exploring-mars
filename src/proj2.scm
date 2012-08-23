;;; Rodrigo Ipince
;;; 6.034 Project 2
;;; Spring 08 - 4/30/08

;;; ===========================================
;;; This file contains four main parts:
;;; 1. Functions detailing the state representation used.
;;; 2. Actions that the rover can perform.
;;; 3. Search strategy and heuristics used.
;;; 4. Helper functions.
;;; 5. Different preset initialization scenarios used for testing.
;;; 6. Actual test cases.
;;; ===========================================


;;; ===========================================
;;; Part 0. Libraries and Globals
;;; ===========================================

;;; Load provided functions (from staff)
(require (lib "init.ss" "6.034"))
(require (lib "compile.ss"))
(define cd current-directory)
(load "load-search.scm")
(load "plan.scm")
(load "rover.scm")

;;; Useful globals
(define *drilling-cost* '(1 1))          ;; (time-cost battery-cost)
(define *communicating-cost* '(0.25 1))  ;; (time-cost battery-cost)
(define *battery-max* 10)
(define *huge-number* 100000)


;;; ===========================================
;;; Part 1. State Representation
;;; ===========================================

;;; A state is represented as a list of assertions. These assertions include:
;;; - (action (go +y)): Indicates the action that led to this state.
;;; - (at (x y)): Indicates the current plane location of the rover (x y).
;;; - (time t): Indicates the time at which we reached this state.
;;; - (battery b): Indicates the current battery charge of the rover.
;;; - (have-sample (x y)): Indicates that the rover holds a sample from location (x y).
;;; - (need-sample (x y)): Indicates that the rover needs a sample from location (x y).
;;; - (have-communicated p): Indicates that the rover has communicated in period p.
;;; - (need-communicate p): Indicates that the rover needs to communicate in period p.

;;; The state-index function is used by the expanded and visited list functions (in
;;; "search-q.scm") to get a description of the state that can be tested with equal? to
;;; determine whether two states are "identical." This turns a state, which is a list, into a
;;; sorted list which is hopefully a unique identifier. In particular, the only assertions
;;; we need to get rid of are the action ones.
(define (state-index state)
  ;; remove the action descriptor, since that's not really part of the state.
  (sort (filter (lambda (a) (not (and (pair? a) (eq? (car a) 'action))))
		state)
	list-alpha<=?))


;;; ===========================================
;;; Part 2. Action Definitions
;;; ===========================================

;;; Actions represent the procedures that the rover may choose to do at a point in time. Each
;;; action takes in the rover's current state and some parameters. If the action results in a
;;; valid state, it returns the new state and the cost incurred in a list of the form
;;; (new-state time-cost). Otherwise, it returns #f.

;;; Go actions enable the rover to move from its current location (kept in the state) to a
;;; neighboring location. The direction argument (x-, x+, y-, y+) specifies which ending
;;; location to go to.
(define (go-action state direction)
  (let* ( ;; get the current location from the state
	 (at-assertion (assoc 'at state))
	 (location (assertion-arg at-assertion))
	 ;; look in the map for the time and battery cost of this action
	 (time-and-battery (lookup-cost location direction))
	 (time-cost (t-cost time-and-battery))	  ; hours
	 (battery-cost (b-cost time-and-battery)) ; units charge
	 (new-state
	  ;; basic resulting state
	  (rover-next-state state time-cost battery-cost)))
    (if new-state
	(list
	 (state-add `(go ,direction)
		    ;; add the new assertions (changed location)
		    `((at ,(offset-location location direction)))
		    ;; delete the old assertions
		    (state-delete (list at-assertion) new-state))
	 ;; we are minimizing time overall, so this is the cost of the action
	 time-cost)
	;; not enough battery to get there, so no new-state.
	#f)))

;;; Drilling enables the rover to take a sample from the given location. The rover must be at
;;; the drilling location and be in need of a sample from that location in order to be able
;;; to take one.
(define (drill-action state location)
  ;; check pre-conditions (rover must be at drilling location and must need a sample)
  (if (state-check `((at ,location) (need-sample ,location)) state)
      ;; we're at the drilling location and need its sample
      (let* ((time-cost (t-cost *drilling-cost*))		; hours
	     (battery-cost (b-cost *drilling-cost*))		; units charge
	     (new-state
	      ;; basic resulting state
	      (rover-next-state state time-cost battery-cost)))
	(if new-state
	    (list
	     (state-add `(drill ,location)
			;; add the new assertions (we now have the sample)
			`((have-sample ,location))
			;; delete old assertions (we don't longer need the sample)
			(state-delete `((need-sample ,location)) new-state))
	     ;; we are minimizing time overall, so this is the cost of the action
	     time-cost)
	    ;; not enough battery to get there, so no new-state
	    #f))
      ;; pre-conditions not true (not at location or not in need of sample)
      #f))

;;; Recharges the rover's battery. This action incorporates all the details involving the
;;; recharging. For instance, if recharge is performed during the night, then the rover will
;;; wait until daylight to recharge, so the resulting state could possibly have a time cost
;;; of more than 10.
(define (recharge-action state)
  ;; only recharge if we're not at max battery
  (if (not (state-check `((battery ,*battery-max*)) state))
      (let* ((time (get-time state))                      ;; current time
            (battery-assertion (assoc 'battery state))    ;; battery assertion
            (charge (assertion-arg battery-assertion))    ;; current battery charge
            (units-to-charge (- *battery-max* charge))    ;; units we need to recharge
            (wasted-time (if (daylight? time)             ;; time wasted at night
                             (if (> units-to-charge (time-to-nightfall time))
                                 12
                                 0)
                             (time-to-daybreak time)))
            (time-cost (+ units-to-charge wasted-time))   ;; total time cost
            (new-state
             (rover-next-state state time-cost 0)))       ;; recharging costs 0 battery units
        (if new-state
            (list
             (state-add '(recharge) ;; action that took us here
                        `((battery ,*battery-max*)) ;; add new assertion (battery full)
                        (state-delete (list battery-assertion) new-state)) ;; delete old battery
	     ;; we are minimizing time overall, so this is the cost of the action
             time-cost)
            ;; cannot get there from our current state (should actually never happen)
            #f))
      ;; no need to recharge
      #f))

;;; Communicate makes the rover communicate if it actually needs to communicate. If it tries
;;; to communicate while not being in a communication window, then it will wait until the
;;; next available window and communicate then.
(define (communicate-action state)
  (let* ((time (get-time state))                 ;; current time
         (period (communication-period time)))   ;; corresponding period
    ;; only communicate if we need to
    (if (state-check `((need-communicate ,period)) state)
        (let* ((comm-cost (t-cost *communicating-cost*))     ;; time cost for communcation
               (wasted-time (time-to-window time))           ;; time spend waiting for window
               (time-cost (+ comm-cost wasted-time))         ;; total time cost
               (battery-cost (b-cost *communicating-cost*))  ;; battery cost
               (new-state (rover-next-state state time-cost battery-cost)))
          (if new-state (list
                         (state-add '(communicate) ;; action that took us here
                                    ;; add communication assertion
                                    `((have-communicated ,period))
                                    ;; remove old assertion (need to communicate)
                                    (state-delete `((need-communicate ,period)) new-state))
                         ;; we are minimizing time overall, so this is the cost of the action
                         time-cost)
              ;; cannot get there from our current state
              #f))
      ;; no need to communicate
      #f)))

;; NOT USED ANYMORE; just left it there for background

;;; The rover can wait until the next communication window. It only makes sense for the rover
;;; to wait while recharging and when all of its tasks except communicating are done.
;;; Otherwise, he would just be wasting time (increasing cost) while not getting any closer
;;; to its goal. The waiting action only considers the second condition since waiting while
;;; charging has already been accounted for in the recharge action. As such, waiting forces
;;; the rover to wait until the next communication window.
;(define (wait-action state)
;  (let ((communicate-assertion (assoc 'need-communicate state)))
;    ;; only wait if we need to communicate
;    (if communicate-assertion
;        (let* ((comm-period (assertion-arg communicate-assertion))
;               (time (get-time state))
;               (period (communication-period time)))
;          ;; only wait if we need to communicate in the future and we're NOT in a comm window
;          (if (and (>= comm-period period) (not (communcation-window? time)))
;              (let* ((time-cost (
;              ;; failed to communicate on some past period
;              #f))
;        ;; no need to communicate, so no need to wait
;        #f)))


;;; ===========================================
;;; Part 3. Planning and Search
;;; ===========================================

;;; This section defines the heuristic used in our A* search.

;;; Returns an estimate for the time left in the mission. This consists of four
;;; components: time spent moving (distance-heuristic), time spent recharging
;;; (battery-heuristic), time spent communicating (communicate-heuristic), and
;;; time spent waiting at night (waiting-time-heuristic).
(define (plan-heuristic state goal-conditions)
  (+ 0
   ;(distance-heuristic state goal-conditions)
   ;(battery-heuristic state goal-conditions)
   (communicate-heuristic state goal-conditions)
   ;(waiting-time-heuristic state goal-conditions)
     ))

;;; Distance heuristic: estimates the remaining time the rover will spend moving.
;;; Returns the product of the minimum moving time-cost times the distance
;;; estimate to the goal (see remaining-distance function in Part 4).
(define (distance-heuristic state goal-conditions)
  (let* ((current-loc (get-location state))           ;; current location
         (goal-loc (get-location goal-conditions))    ;; goal location
         (undrilled-locs (undrilled-locations state)) ;; undrilled locations
         (distance-estimate                           ;; estimated distance
          (remaining-distance current-loc goal-loc undrilled-locs)))
    (* (t-cost *min-cost*) distance-estimate)))

;;; Battery heuristic: estimates the remaining time the rover will spend recharging.
;;; It does so by calculating the amount of battery units needed to complete the
;;; mission and subtracting the current battery charge from those.
(define (battery-heuristic state goal-conditions)
  (let ((undrilled-locations (undrilled-locations state))
        (uncommunicated-periods (uncommunicated-periods state)))
    (max 0 ;; do not return negative estimate
         (-
          ;; estimated battery units needed to complete mission
          (+
           ;; battery spent on remaining distance
           (* (b-cost *min-cost*) (remaining-distance
                                   (get-location state)
                                   (get-location goal-conditions)
                                   undrilled-locations))
           ;; battery spent on remaining drilling locations
           (* (b-cost *drilling-cost*) (length undrilled-locations))
           ;; battery spend communicating
           (* (b-cost *communicating-cost*) (length uncommunicated-periods)))
          ;; minus current battery
          (assertion-arg (assoc 'battery state))))))

;;; Communicate heuristic: estimates the amount of time the rover will spend communicating.
;;; It does so by calculating the amount of communications are still needed to complete the
;;; mission and multiplying that by the communication time-cost. Additionally, if the rover
;;; has failed to communicate on some period, the estimate is a huge number, which will
;;; prevent the search from following such a path.
(define (communicate-heuristic state goal-conditions)
  (let* ((current-time (get-time state))                       ;; current time
         (uncommunicated (uncommunicated-periods state)))      ;; uncommunicated periods
    (if (failed-to-communicate current-time uncommunicated)
        ;; we failed to communicate, so return extremely high number
        *huge-number*
        ;; else return the amount of time we'll spend communicating
        (* (t-cost *communicating-cost*) (length uncommunicated)))))


;;; Wasted time (waiting) heuristic: estimates the time that the rover will have to
;;; wait in the remainder of the mission. Implementing this procedure in a general
;;; way is complicated, which is why we only care if it is the nighttime. In that case,
;;; we calculate the minimum amount of time the rover must wait, which is given by
;;; the time until daybreak minus the current battery times the minimum cost of moving.
(define (waiting-time-heuristic state goal-conditions)
  (let ((current-time (get-time state))                            ;; current time
        (current-battery (assertion-arg (assoc 'battery state))))  ;; current battery
    (if (not (daylight? current-time))
        (max 0
             ;; time to day - max amount of non-wasted hours (rover is moving)
             (- (time-to-daybreak current-time)
                ;; this is the amount max number of hours the rover can move with its
                ;; current battery supply
                (floor (/ current-battery (b-cost *min-cost*)))))
        0)))

;;; The plan procedure is just an A* search with the plan-heuristic defined above.
(define (plan-a* initial-state goal-assertions)
  (search-node-path 
   ;; this takes the goal-assertions directly instead of a goal-test function
   (a*-for-plan initial-state goal-assertions plan-neighbors plan-heuristic 'use-expanded)))


;;; ===========================================
;;; Part 4. Helper Functions
;;; ===========================================

;;; Given a state, returns the time if there is one. Otherwise, returns #f.
(define (get-time state)
  (let ((time-assertion (assoc 'time state)))
    (if time-assertion
        (assertion-arg time-assertion)
        #f)))

;;; Given a state, returns the location if there is one. Otherwise, returns #f.
(define (get-location state)
  (let ((location-assertion (assoc 'at state)))
    (if location-assertion
        (assertion-arg location-assertion)
        #f)))

;;; Given a state, returns a list with the drilling locations yet to be drilled.
(define (undrilled-locations state)
  (undrilled-locations-helper state '()))

;;; Helper for the undrilled-locations procedure
(define (undrilled-locations-helper state locations)
  (let ((undrilled-assertion (assoc 'need-sample state)))
    (if undrilled-assertion
        ;; add location to list, delete assertion from state, and continue
        (undrilled-locations-helper (state-delete (list undrilled-assertion) state)
                                    (cons (assertion-arg undrilled-assertion) locations))
        ;; no more undrilled locations, so return current list
        locations)))

;;; Given a state, returns a list with the periods in which communication is still needed.
(define (uncommunicated-periods state)
  (uncommunicated-periods-helper state '()))

;;; Helper for the uncommunicated-periods procedure
(define (uncommunicated-periods-helper state periods)
  (let ((uncommunicated-assertion (assoc 'need-communicate state)))
    (if uncommunicated-assertion
        ;; add period to list, delete assertion from state, and continue
        (uncommunicated-periods-helper (state-delete (list uncommunicated-assertion) state)
                                      (cons (assertion-arg uncommunicated-assertion) periods))
        ;; no more uncommunicated periods, so return current list
        periods)))

;;; Returns #t if we have failed to communicate when we needed to. Otherwise, returns #f.
(define (failed-to-communicate current-time uncommunicated)
  (let ((current-period (communication-period current-time)))
    (and (not (null? uncommunicated))                     ;; some communications are left
         (< (apply min uncommunicated) current-period)))) ;; first uncommunicated is in past

;;; For each of the remaining drilling locations, this function calculates
;;; the manhattan distance from the current location to the drilling location
;;; and from the drilling location to the goal location. If there are no remaining
;;; drilling locations, it just calculates the manhattan distance from the current
;;; location to the goal location. It then returns the max of these distances.
(define (remaining-distance current-loc goal-loc undrilled-locs)
  (if (and current-loc goal-loc)
      ;; if no undrilled locations
      (if (null? undrilled-locs)
          ;; calculate distance from rover to goal
          (manhattan-distance current-loc goal-loc)
          ;; else calculate max distance from rover to loc + loc to goal
          (apply max
                 (map (lambda (drill-loc)
                        (+ (manhattan-distance current-loc drill-loc)
                           (manhattan-distance drill-loc goal-loc)))
                      undrilled-locs)))
      ;; no current location or goal location (invalid), so return huge estimate
      *huge-number*))

;;; Returns the time there is from the given time to the next communcation window (10 am)
(define (time-to-window time)
  (let ((period (communication-period time)))
    (max 0 (- (+ 10 (* 24 period)) time))))

;;; Time cost
(define t-cost first)

;;; Battery cost
(define b-cost second)


;;; ===========================================
;;; Part 5. Initializing Rover Planning
;;; ===========================================

;;; This sections contains different initializations used for different test cases.

;;; This initialization creates a map with uniform costs of (1 2) with the following
;;; starting and ending conditions:
;;; Initial state: at (0 0), time 0, battery 10, need-sample (1 1),
;;;                need-sample (2 1), need-communicate 0.
;;; Goal state:    at (0 0), have-sample (1 1), have-sample (2 1),
;;;                have-communicated 0.
(define (initialize-rover-planning-1)
  (let ((drill-locations 
	 '((1 1) 
	   (2 1)
	   )))

    (set! *rover-initial-state*
	  `((action ())
	    (at (0 0))			; at lander location
	    (time 0)			; just landed
	    (battery 10)		; full battery charge
	    (need-sample ,(first drill-locations))
	    (need-sample ,(second drill-locations))
	    (need-communicate 0)
	    ))
    (set! *rover-goal*
	  `((at (0 0))
	    (have-sample ,(first drill-locations))
	    (have-sample ,(second drill-locations))
	    (have-communicated 0)
	    ))
	
    (initialize-map)
    ;(add-map-rectangle '(0 0) '(3 3) 5 5)

    (set! *min-cost* '(1 2))		; default cost for motion is minimum

    ;; set the global variable that defines the actions for this domain.
    (set! *actions-and-args*
	  (list
	   (list go-action (combinations '(x- x+ y- y+)))	; 1 argument
	   (list drill-action (combinations drill-locations))	; 1 argument
           (list recharge-action '(()))                         ; no arguments
           (list communicate-action '(()))                      ; no arguments
	   ))))

;;; This initialization creates a map with uniform costs of (2 2) with the following
;;; starting and ending conditions:
;;; Initial state: at (0 0), time 0, battery 10, need-sample (1 1),
;;;                need-sample (2 1), need-sample (3 3), need-communicate 0.
;;; Goal state:    at (0 0), have-sample (1 1), have-sample (2 1),
;;;                have-sample (3 3), have-communicated 0.
(define (initialize-rover-planning-2)
  (let ((drill-locations 
	 '((1 1) 
	   (2 1)
           (3 3)
	   )))

    (set! *rover-initial-state*
	  `((action ())
	    (at (0 0))			; at lander location
	    (time 0)			; just landed
	    (battery 10)		; full battery charge
	    (need-sample ,(first drill-locations))
	    (need-sample ,(second drill-locations))
            (need-sample ,(third drill-locations))
	    (need-communicate 0)
	    ))
    (set! *rover-goal*
	  `((at (0 0))
	    (have-sample ,(first drill-locations))
	    (have-sample ,(second drill-locations))
            (have-sample ,(third drill-locations))
	    (have-communicated 0)
	    ))
	
    (initialize-map)
    ;(add-map-rectangle '(0 0) '(3 3) 5 5)

    (set! *min-cost* '(1 1))		; default cost for motion is minimum

    ;; set the global variable that defines the actions for this domain.
    (set! *actions-and-args*
	  (list
	   (list go-action (combinations '(x- x+ y- y+)))	; 1 argument
	   (list drill-action (combinations drill-locations))	; 1 argument
           (list recharge-action '(()))                         ; no arguments
           (list communicate-action '(()))                      ; no arguments
	   ))))

;;; This initialization creates a map min costs of (1 1), where the rover
;;; is almost surrounded by high-cost area, and with the following
;;; starting and ending conditions:
;;; Initial state: at (0 0), time 0, battery 10, need-sample (3 0),
;;;                need-communicate 0.
;;; Goal state:    at (3 0), have-sample (3 0), have-communicated 0.
(define (initialize-rover-planning-3)
  (let ((drill-locations 
	 '((3 0)
	   )))

    (set! *rover-initial-state*
	  `((action ())
	    (at (0 0))			; at lander location
	    (time 0)			; just landed
	    (battery 10)		; full battery charge
	    (need-sample ,(first drill-locations))
	    (need-communicate 0)
	    ))
    (set! *rover-goal*
	  `((at (3 0))
	    (have-sample ,(first drill-locations))
	    (have-communicated 0)
	    ))
	
    (initialize-map)
    (add-map-rectangle '(-1 1) '(2 2) 5 5)
    (add-map-rectangle '(1 -1) '(2 1) 5 5)
    (add-map-rectangle '(-1 -2) '(2 -1) 5 5)

    (set! *min-cost* '(1 1))		; default cost for motion is minimum

    ;; set the global variable that defines the actions for this domain.
    (set! *actions-and-args*
	  (list
	   (list go-action (combinations '(x- x+ y- y+)))	; 1 argument
	   (list drill-action (combinations drill-locations))	; 1 argument
           (list recharge-action '(()))                         ; no arguments
           (list communicate-action '(()))                      ; no arguments
	   ))))

;;; Initialization given by the staff.
(define (initialize-rover-planning-4)
  (let ((drill-locations 
	 '((1 1) 
	   (2 2)
	   )))

    (set! *rover-initial-state*
	  `((action ())
	    (at (0 0))			; at lander locatin
	    (time 0)			; just landed
	    (battery 10)		; full battery charge
	    (need-sample ,(first drill-locations))
	    (need-sample ,(second drill-locations))
	    ;; communication requirements for a three-day mission
	    (need-communicate 0)
	    (need-communicate 1)
	    (need-communicate 2)
	    ))
    (set! *rover-goal*
	  `((at (0 0))
	    (have-sample ,(first drill-locations))
	    (have-sample ,(second drill-locations))
	    ;; communication requirements for a three-day mission
	    (have-communicated 0)
	    (have-communicated 1)
	    (have-communicated 2)
	    ))
	
    (initialize-map)
    (add-map-rectangle '(1 1) '(3 3) 3 3) ; very slow and draining

    (set! *min-cost* '(2 2))		; default cost for motion is minimum

    ;; set the global variable that defines the actions for this domain.
    (set! *actions-and-args*
	  (list
	   (list go-action (combinations '(x- x+ y- y+)))	; 1 argument
	   (list drill-action (combinations drill-locations))	; 1 argument
           (list recharge-action '(()))                         ; no arguments
           (list communicate-action '(()))                      ; no arguments
	   ))))


;;; ===========================================
;;; Part 6. Testing
;;; ===========================================

(initialize-rover-planning-3)
(time (plan-a* *rover-initial-state* *rover-goal*))