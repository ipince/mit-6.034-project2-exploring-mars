;;;; -*- mode:Scheme -*- ;;;;
(declare (usual-integrations))          ; Just for MIT Scheme compiler

;;; Planning for a simulated Mars exploration robot (for 6.034 by TLP@mit.edu)
;;; MUST BE LOADED AFTER PLAN.SCM

(define *min-cost* '(0.25 0.1))	; (time-cost battery-cost)
(define *map* '())

;;; Global variables to hold the initial state and the goal.  Just for
;;; convenience in testing.
(define *rover-initial-state* #f)
(define *rover-goal* #f)

;;; Initialize global variables.  Edit this for other test cases.
(define (initialize-rover-planning)
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
    (add-map-rectangle '(1 1) '(3 3) 5 5) ; very slow and draining

    (set! *min-cost* '(2 2))		; default cost for motion is minimum

    ;; set the global variable that defines the actions for this domain.
    (set! *actions-and-args*
	  (list
	   (list go-action (combinations '(x- x+ y- y+)))	; 1 argument
	   (list drill-action (combinations drill-locations))	; 1 argument
	   ))))

;;; Update the basic variables (time and battery) for a state
(define (rover-next-state state time-cost battery-cost)
  (let* ( ;; the current battery in state
	 (battery-assertion (assoc 'battery state))
	 (charge (assertion-arg battery-assertion))
	 ;; the current time in state
	 (time-assertion (assoc 'time state))
	 (time (assertion-arg time-assertion)))
    (cond ((> charge battery-cost)
	   (state-add #f		; doesn't matter which action
		      ;; add the new assertions
		      `((battery ,(min 10 (max 0 (- charge battery-cost))))
			(time ,(+ time time-cost)))
		      ;; delete the old assertions
		      (state-delete
		       (if (assoc 'stay state)
			   ;; (stay time) added by actions that don't move the
			   ;; robot, e.g. charge and wait.  Why?
			   (list battery-assertion
				 time-assertion
				 (assoc 'stay state))
			   (list battery-assertion
				 time-assertion))   
		       state)))
	  (else
	   ;; not enough battery to get there.
	   #f))))

;;; A couple of basic actions, you will need several more.

;;; Go actions are defined relative to the current location, so one specifies
;;; only a direction (x-, x+, y-, y+).
(define (go-action state direction)
  (let* ( ;; the current location in state
	 (at-assertion (assoc 'at state))
	 (location (assertion-arg at-assertion))
	 ;; look in the map for the time and battery cost of this action
	 (time-and-battery (lookup-cost location direction))
	 (time-cost (first time-and-battery))	  ; hours
	 (battery-cost (second time-and-battery)) ; units charge
	 (new-state
	  ;; basic resulting state
	  (rover-next-state state time-cost battery-cost)))
    (if new-state
	(list
	 (state-add `(go ,direction)
		    ;; add the new assertions
		    `((at ,(offset-location location direction)))
		    ;; delete the old assertions
		    (state-delete (list at-assertion) new-state))
	 ;; we are minimizing time overall, so this is the cost of the action
	 time-cost)
	;; not enough battery to get there, so no new-state.
	#f)))

;;; Drilling has one argument, the location
(define (drill-action state location)
  ;; check pre-conditions
  (if (state-check `((at ,location) (need-sample ,location)) state)
      ;; We're at the drilling location
      (let* ((time-cost 1)			; hours
	     (battery-cost 1)			; units charge
	     (new-state
	      ;; basic resulting state
	      (rover-next-state state time-cost battery-cost)))
	(if new-state
	    (list
	     (state-add `(drill ,location)
			;; add the new assertions
			`((have-sample ,location))
			;; delete
			(state-delete `((need-sample ,location)) new-state))
	     ;; we are minimizing time overall, so this is the cost of the action
	     time-cost)
	    ;; not enough battery to get there, so no new-state
	    #f))
      ;; pre-conditions not true
      #f))

;;; ===================
;;; Some important functions that need to be modified.

;;; Youl'll need to modify this!!!
(define (plan-heuristic state goal-conditions)
  0)

;;; Hint, you'll want this...
(define (manhattan-distance loc1 loc2)
  (reduce + 0 (map abs (map - loc1 loc2))))

;; You'll need to modify this!!!
(define (state-index state)
  ;; remove the action descriptor, since that's not really part of the state.
  (sort (filter (lambda (a) (not (and (pair? a) (eq? (car a) 'action))))
		state)
	list-alpha<=?))

;;; ===================
;;; Some helper functions you probably won't need to modify.

(define assertion-arg second)		; get the arg of assertion

;;; Produce a new (x y) location given an initial location and direction of motion.
(define (offset-location location direction)
  (let ((x (first location))
	(y (second location)))
    (cond ((eq? direction 'x+) (list (+ x 1) y))
	  ((eq? direction 'x-) (list (- x 1) y))
	  ((eq? direction 'y+) (list x (+ y 1)))
	  ((eq? direction 'y-) (list x (- y 1)))
	  (else
	   (display* "Unknown direction for motion: " direction)
	   ;; unknown direction, don't move
	   location))))

;;; Find map entry at location
(define (lookup-map-entry location map)
  (let ((entry (assoc location map)))
    (if entry
	(cdr entry)
	#f)))

;;; Finds the cost for the location and direction, if it's not specified then
;;; return default (min cost).
(define (lookup-cost location direction)
  (let ((map-entry (lookup-map-entry location *map*))
	(index (second (assoc direction '((x- 0) (x+ 1) (y- 2) (y+ 3))))))
    (if map-entry			; entry could be missing
	(or (list-ref map-entry index)
	    ; cost can be #f
	    *min-cost*)
	*min-cost*)))

(define (initialize-map)
  (set! *map* '())
  )

;; point is of the form ((x y) cost-x- cost-x+ cost-y- cost-y+)
;; cost is of the form (time-cost battery-cost)
;; note that the costs are "one way".
(define (add-point-map point)
  (set! *map* (cons point *map*))
  )

;;; Add a rectangle in the map with uniform costs for all directions
;;; The costs should not be below the *min-cost*, but this is not enforced here.
(define (add-map-rectangle low-corner high-corner time-cost battery-cost)
  (let* ((cost (list time-cost battery-cost))
	 (cost-list (list cost cost cost cost)))
    (do ((x (first low-corner) (+ x 1)))
	((> x (first high-corner)))
      (do ((y (second low-corner) (+ y 1)))
	  ;; fixed bug on 4/18/07 ... was (> y (first high-corner))
	  ((> y (second high-corner)))
	(add-point-map (cons (list x y) cost-list))
	))))

;;; These may come in handy...

;; Day?  Nighttime begins at 18 hours.
(define (daylight? time)
  (if (>= time 24)
      (daylight? (- time 24))
      (and (>= time 6) (< time 18))))

;; In the window to communicate?
(define (communication-window? time)
  (if (>= time 24)
      (communication-window? (- time 24))
      (and (>= time 10) (<= time 12))))

;; Index (0 based) to the next communication period
(define (communication-period time)
  (define (loop t i)
    (if (> t 12)			; after the end of the window
	(loop (- t 24) (+ i 1))
	i))
  (loop time 0))

(define (time-to-nightfall time)
  (if (>= time 24)
      (time-to-nightfall (- time 24))
      (if (<= time 18)
	  ;; time to today's nightfall
	  (- 18 time)
	  ;; after nightfall today, time to midnight + 18
	  (+ 18 (- 24 time)))))

(define (time-to-daybreak time)
  (if (>= time 24)
      (time-to-daybreak (- time 24))
      (if (<= time 6)
	  (- 6 time)
	  ;; next daybreak, time to midnight + 6
	  (+ 6 (- 24 time)))))

;;; ===================
;; initialize the example by computing the argument lists for the actions and
;; the map.
(initialize-rover-planning)

