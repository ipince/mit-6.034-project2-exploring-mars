;;;; -*- mode:Scheme -*- ;;;;

;;; GENERALIZED SEARCH PROCEDURE (for 6.034 by TLP@mit.edu)

(declare (usual-integrations))          ; Just for MIT Scheme compiler

;;; One important implementation note.  The pending list is
;;; implemented in "message passing style".  That is, as a function
;;; that takes a symbol (indicating the operation) and some arguments.

;;; A global variable to keep track of the amount of work done.
(define *number-of-search-steps* 0)
;;; Controls amount of printing, set to #f to limit printing
(define *verbose* #t)

;;; data structure for search-node.  Defines accessor and modifier
;;; functions: search-node-cost, set-search-node-cost!, etc.
;;; If you need to display a search-node, use the search-node-display
;;; function defined later in this file.  The regular Scheme display 
;;; function does not print structures.
(define-structure (search-node)
  cost actual estimate state predecessor id)

;;; This allows implementing most of the search strategies
;;; (depth-first, etc.) by specifying the appropriate type of pending list
;;; function. Returns either a winning search-node or #f.  This
;;; assumes a single goal state, generalization to a list is trivial.

(define (search goal                    ; goal state
                successors              ; successor function
                pending                 ; pending list function (the Q)
                expanded                ; expanded list function (or #f)
                visited                 ; visited list function (or #f)
                )

  (cond
   ((pending 'empty?)                           ; Failed, at least be cute...
    (display* "Cain't get thar from heah.")
    #f)
   (else
    (let ((current (pending 'next)))            ; get (and remove) node to expand

      ;; Show some status information
      (if *verbose*                     ; only in verbose mode.
          (search-node-display current "Current node: "))
      (set! *number-of-search-steps* (1+ *number-of-search-steps*))
      (cond ((= 0 (remainder *number-of-search-steps* 100))
             (pending 'summary)))

      (cond ((and expanded
                  (expanded 'member (search-node-state current)))
             (if *verbose* (display* "Already expanded."))
             (search goal successors pending expanded visited))
            ((if (procedure? goal)      ; are we there?
                 (goal (search-node-state current))
                 (equal? (search-node-state current) goal))
             (pending 'summary)         ; print a summary of pending list
             (search-node-display current " Final: ") ; display node
             (let ((path (search-node-path current)))
               (display* " Path length = " (length path))
               (display* " Path = " (map state-name path)))
             current)
            (else 
             ;; pending still has entries in it and we haven't found goal
             ;; yet, so expand current node and merge results into pending
             ;; and update the expanded and visited lists.
             (search-update current (successors current) pending expanded visited)
             (search goal successors pending expanded visited))))
    )))

;;; This is the simple version (uses more space and cannot cope with inconsistent heuristic).
(define (search-update current          ; current search node
                       neighbors        ; list of neighbor nodes
                       pending          ; pending list function (the Q)
                       expanded         ; the expanded list function (or #f)
                       visited          ; the visited list function (or #f)
                       )
  ;; We've expanded current, so add it to expanded list.
  (if expanded (expanded 'add (list current)))
  (cond ((or expanded visited)
         (let ((kept-nodes
                (filter (lambda (node)
                          (not ((or visited expanded) 'member (search-node-state node))))
                        neighbors)))
           ;; Add the new nodes to the pending (and visited) list
           (pending 'add kept-nodes)
           (if visited (visited 'add kept-nodes))))
        (else
         ;; just add to pending.
         (pending 'add neighbors)))
  )

;;; SEARCH-NODE OPERATIONS

;; Extend a path to the neighbors of the node's state. Returns a list of
;; new search-nodes (with cost given by the path length (if a link-cost function 
;; is given) plus the heuristic function value, if one is given).   

(define (extend-node node               ; a search node
                     neighbors          ; a function of a state, returns list of neighbor links
                     link-cost          ; a function of a link, returns cost
                     heuristic          ; a function of a state, returns estimate
                     )
  (define (loop neighbor-links)
    (let* ((nbor-link (if (null? neighbor-links) #f (first neighbor-links)))
           (nbor-state (neighbor-state nbor-link)))
      (cond ((not nbor-link) '())
            ((predecessor? nbor-state node)
             ;; skip this state - we don't want to re-visit it again.
             (loop (rest neighbor-links)))
            (else 
             ;; construct new node and cons it to the list.
             (cons
              (let ((new-node 
                     (make-search-node 0 0 0 nbor-state node #f)))
                ;; Fill in the fields of the new-node
                (set-search-node-state! new-node nbor-state)
                (set-search-node-predecessor! new-node node)
                (search-node-update-cost!
                 new-node               
                 (if link-cost
                     ;; link cost given, add to the actual path cost so far.
                     (+ (link-cost nbor-link) (search-node-actual node))
                     ;; no link cost given, so set actual cost to 0
                     0)
                 (if heuristic
                     ;; heuristic given, call it on the neighboring state
                     (heuristic nbor-state)
                     ;; no heuristic given, use 0
                     0))
                new-node)
              (loop (rest neighbor-links)))))))
  (loop (neighbors (search-node-state node))))

;;; Returns #t if state is in the predecessor chain for node.
(define (predecessor? state node)
  (cond ((equal? state (search-node-state node)) #t)
        ((search-node? (search-node-predecessor node))
         (predecessor? state (search-node-predecessor node)))
        (else #f)))

;;; Construct a path (a list of states) with the first state first
;;; from a search-node.
(define (search-node-path node)
  (define (loop sn)                     ; construct reversed list
    (if sn
        (cons (search-node-state sn) 
              (loop (search-node-predecessor sn)))
        '()))
  (reverse (loop node)))

;;; Update the costs in a node.  If the input is #f, leave cost alone
(define (search-node-update-cost! node new-actual new-estimate)
  (if new-actual
      (set-search-node-actual! node new-actual))
  (if new-estimate
      (set-search-node-estimate! node new-estimate))
  (if (or new-actual new-estimate)
      (set-search-node-cost! 
       node (+ (search-node-actual node) (search-node-estimate node))))
  node)
                             
(define (search-node-display node . message)
  (display* (if (null? message) 
                " Search node: "
                (car message))
            (state-name (search-node-state node))
            " Cost= " 
            (search-node-cost node)
            " Actual Cost= " 
            (search-node-actual node)
            " Estimate= " 
            (search-node-estimate node)))

(define (make-start-node cost state)
  (make-search-node cost 0 cost state #f #f))

;;;; THE ACTUAL SEARCH METHODS
;;; start and goal are states
;;; neighbors is a function that generates a list of links to neighbors
;;; args is a list of symbols (use-visited, use-expanded, etc)

(define (depth-first start              ; state
                     goal               ; state
                     neighbors          ; a function of a state, 
                                        ; returns list of neighbor links 
                     . args             ; optional args (use-visited,
                                        ; use-expanded, use-wt, etc)
                     )
  (verify-args args)
  (let* ((start-node (make-start-node 0 start))
         (visited (if (member 'use-visited args)
                      (make-state-list start-node)
                      #f)))

    ;; The successors function to be used in the search.
    (define (successors search-node)
      ;; no costs are given.
      (extend-node search-node neighbors #f #f))

    (set! *number-of-search-steps* 0)   ; initialize count

    (search
     goal                               ; goal state
     successors                         ; successors
     (make-stack start-node) 
     #f
     visited
     )))

(define (best-first start               ; state
                    goal                ; state 
                    neighbors           ; a function of a state, 
                                        ; returns list of neighbor links
                    h                   ; a function of a current state and goal state
                    . args              ; optional args (use-visited,
                                        ; use-expanded, use-wt, etc)
                    )

  ;; extend-node expects a function of a single state argument.
  (define (heuristic state) 
    ;; in general, the heuristic value will depend on the goal state
    (h state goal))

  (verify-args args)
  (let* ((start-node (make-start-node (heuristic start) start))
         (visited (if (member 'use-visited args)
                      (make-state-list start-node)
                      #f)))

    ;; The successors function to be used in the search.
    (define (successors search-node)
      ;; neighbor-cost is not given to extend-node, since queue is
      ;; sorted by total cost and we want to use only the heuristic
      ;; value for best-first.
      (extend-node search-node neighbors #f heuristic))

    (set! *number-of-search-steps* 0)   ; initialize count

    (search
     goal                               ; goal state
     successors
     ;; a priority queue that will return the "best" node (by heuristic)
     ((if (member 'use-wt args) make-wt-pq make-pq) start-node) 
     #f
     visited
     )))

;;; Below, we allow for using expanded and visited, which allows us to
;;; detect when we reach a state on the pending list by a shorter path
;;; or with a better estimate (see search-update.scm).  This will not
;;; be useful for uniform-cost but it will be useful for A* with an
;;; inconsistent heuristic.

(define (uniform-cost start             ; state
                      goal              ; state
                      neighbors         ; a function of a state,
                                        ; returns list of neighbor links
                      . args            ; optional args
                      )
  (verify-args args)
  (let* ((start-node (make-start-node 0 start))
         (expanded (if (member 'use-expanded args)
                       (make-state-list)
                       #f))
         (visited (if (member 'use-visited args)
                      (make-state-list start-node)
                      #f)))

    ;; The successors function to be used in the search.
    (define (successors search-node)
      ;; only link-cost is given to extend-node
      (extend-node search-node neighbors neighbor-cost #f))

    (set! *number-of-search-steps* 0)   ; initialize count

    (search
     goal                               ; goal state
     successors
     ;; a priority queue that will return the "best" node
     ((if (member 'use-wt args) make-wt-pq make-pq) start-node) 
     expanded
     visited
     )))

(define (verify-args args)
  (for-each (lambda (arg)
              (or (memq arg '(use-expanded use-visited use-wt))
                  (error "Unknown argument: " arg)))
            args))

;;; STATES

;;; This is a very primitive implementation of states, modeled on the
;;; examples in the notes.  Look at the puzzle code to see a much more
;;; realistic implementation.

;;; Given a state returns a list of graph links, characterized by a
;;; neighbor state and the link cost.
(define (state-neighbors state graph)
  (let ((ans (assoc state graph)))
    (if ans
        (second ans)
        (error "state-neighbors:Unknown state" state))))

;;; Access the two components of a link, the neighbor-state and the cost.
(define (neighbor-state link) (and link (first link)))
(define (neighbor-cost link) (and link (second link)))

;; In general, the heuristic value may depend on the goal state but,
;; here, we simply lookup heuristic value in a table specified for a
;; particular goal.
(define (state-heuristic-value state goal heuristic-values)
  (let ((ans (assoc state heuristic-values)))
    (if ans
        (second ans)
        (error "state-heuristic-value:Unknown state" state))))

;;; Returns something that can be compactly printed as the name of a state.
(define (state-name state) state)
;;; Returns something that can be used to index the state in a hash-table.
(define (state-index state) state)

;;; a little test network (the one from the on-line material).  Each
;;; sublist is (state connected-states) - this is unidirectional.
(define *graph-1* 
  '((s ((a 2) (B 5)))
    (A ((C 2) (D 4)))
    (B ((D 1) (G 5)))
    (C ())
    (D ((C 3) (G 2)))
    (G ())))

;; Trivial heuristic values for fixed goal state G
(define *heuristic-values-1*
  '((a 2) (b 3) (c 1) (d 1) (s 4) (g 0)))

;;; Another network with a very inconsistent heuristic.  
;;; Using only extended list in A* gives the wrong answer.
(define *graph-2* 
  '((s ((a 1) (B 2)))
    (A ((C 1)))
    (B ((C 2)))
    (C ((G 100))
    (G ()))))

(define *heuristic-values-2a*           ; inconsistent
  '((a 100) (b 1) (c 90) (s 90) (g 0)))
(define *heuristic-values-2b*           ; consistent
  '((a 100) (b 88) (c 90) (s 90) (g 0)))

(define *graph-3*
  '((s ((a 1) (b 1) (c 10)))
    (a ((d 1) (s 1)))
    (b ((d 1) (s 1)))
    (c ((h 1) (s 10)))
    (d ((a 1) (b 1) (e 1) (f 1)))
    (e ((d 1) (f 1)))
    (f ((d 1) (e 1)))
    (h ((c 1) (i 1)))
    (i ((h 1) (j 1)))
    (j ((i 1) (g 1)))
    (g ((j 1)))))

(define *heuristic-values-3*
  '((s 10) (a 5) (b 11) (c 1) (d 7) (e 6) (f 5) (h 3) (i 2) (j 1) (g 0)))


