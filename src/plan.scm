;;;; -*- mode:Scheme -*- ;;;;
(declare (usual-integrations))          ; Just for MIT Scheme compiler

;;; Planning via state-space search (for 6.034 by TLP@mit.edu)
;;; MUST BE LOADED AFTER SEARCH CODE.

;;; This is primarily a front end for the Serch code to do planning, illustrated
;;; with a simple example about buying stuff in stores.

;;;==================
;;; Top-level functions

;;; To get a plan using uniform-cost
;;; (plan-uc *store-initial-state* *store-goal*)
(define (plan-uc initial-state goal-assertions)
  (search-node-path 
   (uniform-cost initial-state (goal-test goal-assertions) plan-neighbors)))

;;; To get a plan using A*
;;; (plan-a* *store-initial-state* *store-goal*)
(define (plan-a* initial-state goal-assertions)
  (search-node-path 
   ;; this takes the goal-assertions directly instead of a goal-test function
   (a*-for-plan initial-state goal-assertions plan-neighbors plan-heuristic)))

;;;==================
;;; Simple example domain

;;; Global variables to hold the initial state and the goal.  Just for
;;; convenience in testing.
(define *store-initial-state* #f)
(define *store-goal* #f)

;;; Some background information that never changes, so might as well leave it
;;; out of the state.
(define *store-directory*
  '((sells SM milk)
    (sells SM bread)
    (sells HW drill)))

;;; Initialize global variables.  Edit this for other test cases.
(define (initialize-store-planning)
  ;; these are local variables
  (let ((places '(home SM HW))
	(stores '(SM HW))
	(items '(drill milk bread)))

    ;; A state is simply a list of assertions.  Every state keeps track of the
    ;; action that created this state, so the assertion (action ...) is always
    ;; present.
    (set! *store-initial-state*
	  '((action ())			; no action, since initial state
	    (at home)
	    ))

    ;; This is not a state, exactly, it's a required SUBSET of a goal state.
    (set! *store-goal*			; set of conditions for the goal
	  '((have milk)
	    (have drill)
	    (at home)))

    ;; set the global variable that defines the actions for this domain.
    (set! *actions-and-args*
	  (list
	   (list drive-action (combinations places places))
	   (list buy-action (combinations  items stores))))))

;;; An action is a function that takes a state and some arguments and produces
;;; the resulting state (or #f if the pre-conditions are not met).  It deletes
;;; some assertions and adds new ones to produce a new state.
(define (drive-action state from to)
  (if (and (not (equal? from to))		 ; non-trivial
	   (state-check `((at ,from)) state))	 ; we're at from location
      (list
       (state-add `(drive ,from ,to) 
		  `((at ,to))			 ; now we're at the to location
		  (state-delete `((at ,from))	 ; will also delete the action
				state))
       ;; cost, could make it depend on the from-to pair
       1)
      ;; pre-conditions not true
      #f))

(define (buy-action state item store)
  (if (and (not (state-check `((have ,item)) state))	; don't have it yet
	   (state-check `((at ,store)) state)		; we're at the store
	   (state-check `((sells ,store ,item)) *store-directory*) ; store sells it
	   )
      (list
       (state-add `(buy ,item ,store) `((have ,item))
		  (state-delete '()	; will also delete the action
				state))
       ;; cost, could depend on the item-store pair
       1)
      ;; pre-conditions not true
      #f))

;;;==================
;;; Key search interface functions for planning

;;; This argument holds lists of actions and all the possible combinations of
;;; arguments, see initialize-store-planning to see how it is initialized.
(define *actions-and-args* #f)

;;; The neighbors function used in the Search code returns a list of (state
;;; cost) entries, which it gets by calling the actions for all the possible
;;; combinations of arguments.
(define (plan-neighbors state)
  (map-append 
   (lambda (action-and-args)
     (let ((action (first action-and-args))
	   (args-list (second action-and-args)))
       (collect (lambda (args) (apply action state args)) args-list)))
   *actions-and-args*))

;; In general, this heuristic will have to be redefined for the specific type of
;; planning problem.  This one works for a large variety of problems where the
;; actions have the same cost and each action fully specifies the resulting
;; state (no relative/conditional actions).

(define *max-h-depth* 10)		; don't go too deep

(define (plan-heuristic state goal-conditions)
  ;; Keep track of how many levels of actions are necessary before the goal
  ;; conditions are present and return that.
  (define (loop result i)
    (display* result)
    (if (or (state-check goal-conditions result) ; check the goal conditions
	    (>= i *max-h-depth*))
	i
	(loop (plan-union-neighbors result) (+ 1 i))))
  ;; start with state and 0 estimate
  (loop state 0))

;;; This returns a single list of all the assertions produced by applying all
;;; the actions that can be applied in state.  This is used in the HSP
;;; heuristic, which is a lower bound on the number of actions required to
;;; achieve the goal.  Note, that it ignores the cost of the actions, assuming
;;; they are all cost = 1.
(define (plan-union-neighbors state)
  (fluid-let ((*state-delete* #f)) 	; disable deletions
    ;; union the results for all action types
    (reduce union '()
	    ;; compute the result (union of states) for each action type
	    (map (lambda (action-and-args)
		   (display* action-and-args)
		   (union-action-results 
		    (first action-and-args)    ; action function
		    (second action-and-args)   ; list of argument lists
		    state))
		 *actions-and-args*))))

;;; A* definition
(define (a*-for-plan start goal neighbors h . args)

  (define (heuristic state) 
    ;; in general, the heuristic value will depend on the goal assertions directly.
    (h state goal))

  (verify-args args)
  (let* ((start-node (make-start-node (heuristic start) start))
	 (expanded (if (member 'use-expanded args)
		       (make-state-list)
		       #f))
	 (visited (if (member 'use-visited args)
		      (make-state-list start-node)
		      #f)))

    ;; the successors function to be used in the search.
    (define (successors search-node)
      ;; both link-cost and heuristic is given to extend-node
      (extend-node search-node neighbors neighbor-cost heuristic))

    (set! *number-of-search-steps* 0)	; initialize count

    (search
     (goal-test goal)			; goal test, made from goal assertions
     successors
     ;; a priority queue that will return the "best" node
     ((if (member 'use-wt args) make-wt-pq make-pq) start-node) 
     expanded
     visited
     )))

;;; The state-index function is used by the expanded and visited list functions
;;; (see make-state-list in "search-q.scm") to get a description of the state
;;; that can be tested with equal? to determine whether two states are
;;; "identical".  This turns a state which is a list, into a sorted list which
;;; is hopefully a unique identifier.

;; In general, this may have to be redefined for the specific type of planning
;; problem.

(define (state-index state)
  ;; remove the action descriptor, since that's not really part of the state.
  (sort (filter (lambda (a) (not (and (pair? a) (eq? (car a) 'action))))
		state)
	list-alpha<=?))

;;; ===================
;;; Below are some helper functions

;; Returns #t when all the assertions are present in the state.
(define (state-check assertions state)
  (for-all? assertions (lambda (a) (member a state))))

;; Returns a function that can be used as a goal test
(define (goal-test assertions)
  (lambda (state) (state-check assertions state)))

;; Returns a state with the new assertions at the front.  It includes an action assertion.
(define (state-add action assertions state)
  (if action
      (cons `(action ,action) (append assertions state))
      (append assertions state)))

(define *state-delete* #t)

;; Returns a new state that is missing the assertions given and also missing the
;; action assertion.  When the *state-delete* variable is #f, it just returns
;; the state.
(define (state-delete assertions state)
  (if *state-delete* 
      (filter (lambda (a) (not (or (member a assertions)
				   (and (pair? a) (eq? (car a) 'action)))))
	      state)
      ;; don't delete anything, simply save the state
      state))

;; Union all the states that result from applying an action in all possible ways
;; in state.  It ignores the costs of the actions.
(define (union-action-results action action-args state)
  (if (null? action-args)
      '()
      (let ((state-cost (apply action state (car action-args))))
	(display* action (car action-args) state-cost)
	(if state-cost
	    (union (first state-cost)	; look at state, ignore cost
		   (union-action-results action (cdr action-args) state))
	    (union-action-results action (cdr action-args) state)))))

;;; Computes the union of two lists, this is like append but it removes
;;; duplicates.  Might be ok to just use append.
(define (union x y)
  (if (null? x)
      y
      (if (member (car x) y)
	  (union (cdr x) y)
	  (cons (car x) (union (cdr x) y)))))

;; returns all the combinations of the input lists 
;; (combinations '(1 2) '(3 4)) -> ((1 3) (1 4) (2 3) (2 4))
(define (combinations . args)

  (define (loop list-of-args)
    (if (null? list-of-args)
	'(())
	(map-append (lambda (old-combo)
		      (map (lambda (new-elt) (cons new-elt old-combo))
			   (car list-of-args)))
		    (loop (cdr list-of-args)))))

  (loop args))

;; maps fn on the elements of l and returns the appended results.
(define (map-append fn l)
  (if (null? l) 
      '()
      (append (fn (car l))
	      (map-append fn (cdr l)))))

;; calls fn on the elements of l and returns the list of values that are not #f.
(define (collect fn l)
  (if (null? l) 
      '()
      (let ((value (fn (car l))))
	(if value
	    (cons value (collect fn (cdr l)))
	    (collect fn (cdr l))))))

;;; These functions serve to "standardize" the representation of states so we
;;; can quickly find whether we've visited them by using a hash-table.

;;; Tests whether list1 should be sorted alphabetically before list2.
(define (list-alpha<=? list1 list2)
  (define (loop l1 l2)
    (cond ((and (null? l1) (null? l2))	; we're done with no differences
	   #t)
	  ((null? l1)			; first list is shorted, so <=
	   #t)
	  ((null? l2)			; second list is shorter
	   #f)
	  ((alpha<? (car l1) (car l2))	; first element is <, so <=
	   #t)
	  ((equal? (car l1) (car l2))	; first element is =, so keep looking
	   (loop (cdr l1) (cdr l2)))
	  (else 			; first element is >, so not <=
	   #f)))
  ;; Do the comparison on the leaves of the lists
  (loop (leaves list1) (leaves list2)))

;;; A list of the leaves of a tree:
;;; ((at home) (action (go home SM))) -> (at home action go home SM)
(define (leaves x)
  (if (null? x)
      '()
      (if (pair? x)
	  (append (leaves (car x)) (leaves (cdr x)))
	  (list x))))

;;; The leaves are either symbols or numbers, compare them.
(define (alpha<? x1 x2)
  (cond ((and (symbol? x1) (symbol? x2))
	 (string<? (symbol->string x1) (symbol->string x2)))
	((and (number? x1) (number? x2))
	 (< x1 x2))
	((if (number? x1) (symbol? x2))
	 #t)
	(else
	 #f)))

;;; Simulate a sequence of actions
(define (simulate initial-state actions)
  (display* initial-state)
  (if (or (null? actions) (not initial-state))
      initial-state
      (simulate
       ;; actions return (state cost) or #f if not applicable.
       (first
	(or (apply (car (car actions))
		   initial-state
		   (cdr (car actions)))
	    '(#f)))
       (cdr actions))))

;;; ===================
;; initialize the example by computing the argument lists for the actions.
(initialize-store-planning)

(display* "--------")
(display* "Example of a plan.")
(display* "--------")

;;; A sequence of actions to solve the simple store problem.
(simulate *store-initial-state*
	  `((,drive-action home SM)
	    (,buy-action milk SM)
	    (,drive-action SM HW)
	    (,buy-action drill HW)
	    (,drive-action HW home)))
