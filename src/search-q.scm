;;;; -*- mode:Scheme -*- ;;;;

;;; SEARCH PROCEDURES (for 6.034 by TLP@mit.edu)

(declare (usual-integrations))          ; Just for MIT Scheme compiler

;;; This defines several versions of a search-Q, each of which is
;;; used to implement different types of search algorithms.
;;; The operations that a search queue must implement are:

;;; (Q 'empty?): return #t or #f
;;; (Q 'next): return a node and removes it from the search-Q
;;; (Q 'add <list of nodes>): add the nodes to the queue
;;; (Q 'remove node): remove the nodes form the queue
;;; (Q 'count): return a count of nodes
;;; (Q 'summary): prints a summary of work done

;;; There are several important variants on search-Qs.
;;; FIFO (first in, first out) - queue (implemented as a list)
;;; LIFO (last in, first out) - stack (implemented as a list)
;;; PQ - priority queue (implemented as a list or weight-balanced tree)

;;; We also define state-lists, that keep track of a node associated 
;;; with a given state.  These are implemented using hash-tables.


;;; LIST implementation of LIFO
(define (make-stack . initial)  ; lifo
  (make-list-search-q 'stack initial))

;;; LIST implementation of FIFO
(define (make-queue . initial)          ; fifo
  (make-list-search-q 'queue initial))

;;; LIST implementation of Priority Queue
(define (make-pq . initial)             ; pq
  (make-list-search-q 'pq initial))

(define (make-list-search-q type initial)
  (let* ((entries '())
         (node-id 0)
         (expansions 0)
         (fun
          (lambda (op . args)
            ;;(print 'Q: op args entries)
            (let ((arg (if (null? args) #f (car args))))
              ;;(display* op arg entries)
              (cond
               ((eq? op 'empty?)        ; no arg
                ;; Test for no entries.
                (null? entries))

               ((eq? op 'remove)        ; arg is node
                (if (memq arg entries)
                    ;; take it out of the entries
                    (set! entries (delq arg entries))
                    (error "Node is not in queue:" arg)))

               ((eq? op 'next)          ; no arg
                (if (null? entries)
                    (error "Search queue is empty."))
                ;; If a priority queue, make sure best element is in front.
                (if (eq? type 'pq)
                    ;; Move the best entry (least cost) to the front.
                    (set! entries (move-best-entry-to-front entries)))
                ;; remove first element and return it.
                (let ((next (first entries)))
                  (set! entries (rest entries)) ; remove first entry
                  next))

               ((eq? op 'add)           ; arg is list of nodes
                ;; every time a succesful expansion happens, add is called 
                ;; (even if with empty list).
                (set! expansions (1+ expansions)) ; keep track of amount of work
                (for-each 
                 (lambda (n)
                   ;; add a unique id to each node
                   (set-search-node-id! n node-id)
                   (set! node-id (1+ node-id)))
                 arg)
                ;; update the entries
                (set! entries
                      (cond
                       ((eq? type 'stack)
                        (append arg entries))
                       ((or (eq? type 'queue) (eq? type 'pq))
                        ;; append is very inefficient since it copies
                        ;; the list of entries.  append! is done by
                        ;; side-effect on the last cons-cell with no
                        ;; copying.  However, it still has to find the
                        ;; end each time, we should really keep a
                        ;; pointer to the last cons cell in entries.
                        (append! entries arg))
                       (else
                        (error "Unknown type of search-Q:" type)))))

               ((eq? op 'count)         ; no arg
                ;; could keep count as we go, but we don't do this very often
                (length entries))

               ((eq? op 'summary)       ; no arg
                (display* " Length of queue= " (length entries)
                          " Cost of first node= " 
                          (if (null? entries) #f (search-node-cost (first entries)))
                          " Expansions = " expansions
                          " Nodes added = " node-id
                          ))

               (else
                (error "Unknown operation for:" type op args))))
            )))
    (fun 'add initial)
    fun
    ))

;;; The elements of a state-list are search-nodes but they are indexed by the
;;; state-index function.
;;; The operations are:
;;; (S 'member state): returns node corresponding to that state or #f
;;; (S 'remove node): remove a node from list
;;; (S 'add <list of nodes>): add nodes to list
;;; (S 'count): returns number of nodes in list

(define (make-state-list . initial)
  (let* ((state-list (make-equal-hash-table))
         (fun
          (lambda (op . args)
            (let ((arg (if (null? args) #f (car args))))
              (cond

               ((eq? op 'member)        ; arg is state, returns node
                (hash-table/get state-list (state-index arg) #f))

               ((eq? op 'remove)        ; arg is node
                (hash-table/remove! state-list 
                                    (state-index (search-node-state arg)))
                )

               ((eq? op 'add)           ; arg is list of nodes
                (for-each 
                 (lambda (n)
                   (if (hash-table/get state-list 
                                       (state-index (search-node-state n))
                                       #f)
                       (display* "Adding duplicate state: " 
                                 (state-index (search-node-state n))))
                   ;; put in the hash table
                   (hash-table/put! state-list
                                    (state-index (search-node-state n))
                                    n))
                 arg))

               ((eq? op 'count)
                (hash-table/count state-list))

               ((eq? op 'hash-table)    ; get the hash table (for debugging)
                state-list)

               (else
                (error "Unknown operation for state-list:" op args))))
            )))
    (fun 'add initial)
    fun
    ))


;;; Modify list1 by replacing the last null cdr with a pointer to list2.
(define (append! list1 list2)
  (define (loop l1)
    (if (null? (cdr l1)) 
        (set-cdr! l1 list2)
        (loop (cdr l1))))
  (cond ((null? list1) list2)           ; special case - null list1.
        (else (loop list1) list1)))

;;; Two different ways of moving the best element to the front.  Find
;;; and move the best element or sort the whole list.

;; Do a linear scan and move the best element to the front
(define (move-best-entry-to-front entries)
  (define (loop p best-p best-cost)
    (cond ((null? (rest P))             ; no more nodes
           (cond ((eq? (cdr best-P) entries)
                  ;; the first node is the best one, so no changes
                  entries)
                 (else
                  ;; best-P = (x best-node y ...)
                  (let ((best-node (second best-P)))
                    ;; eliminate the best-node from where it is now
                    (set-cdr! best-P (cddr best-P))
                    ;; add it back to the front of the original list
                    (cons best-node entries)))))
          ;; the cost of the current node is better than the best so far
          ((< (search-node-cost (second P)) best-cost)
           (loop (rest P) P (search-node-cost (second P))))
          ;; keep going
          (else
           (loop (rest P) best-P best-cost))))
  (let ((extended (cons 'anchor entries)))
    ;; add a symbol in front so we can easily remove an element from
    ;; the list, since we need to have a pointer to the cell that
    ;; points to the cell to be removed (see the set-cdr! above).
    (loop (rest extended) extended (search-node-cost (first entries))))
  )

;; Sort the paths based on the heuristic value (and node id as a secondary key).  
;; This gives us a predictable total order.
(define (move-best-entry-to-front nodes)
  (sort nodes (lambda (x y) 
                (or (< (search-node-cost x) (search-node-cost y))
                    (and (= (search-node-cost x) (search-node-cost y))
                         (< (search-node-id x) (search-node-id y)))))))

;;; Using MIT Scheme's Weight-Balanced Trees (WT-Tree)

(load-option 'wt-tree)

;;; Comparison function for WT.  It uses a cons of the cost and the
;;; node id to construct a total order on the nodes.
(define (search-node-< c1 c2)
  (or (< (car c1) (car c2))
      (and (= (car c1) (car c2))
           (< (cdr c1) (cdr c2)))))

(define (wt-tree/extract-min! wt)
  (let ((next (wt-tree/min-datum wt)))
    (wt-tree/delete-min! wt)            ; remove first entry
    next))

(define (make-wt-pq . initial)
  (let* ((wt (make-wt-tree (make-wt-tree-type search-node-<)))
         (node-id 0)
         (expansions 0)
         (fun
          (lambda (op . args)
            (let ((arg (if (null? args) #f (car args))))
              (cond
               ((eq? op 'empty?)        ; no arg
                ;; Test for no entries.
                (wt-tree/empty? wt))

               ((eq? op 'remove)        ; arg is node
                (let ((key (cons (search-node-cost arg) 
                                 (search-node-id arg))))
                  ;; take it out of the entries
                  (wt-tree/delete wt key)
                  ))

               ((eq? op 'next)          ; no arg
                (if (wt-tree/empty? wt)
                    (error "Search queue is empty."))
                ;; remove first element and return it.
                (wt-tree/extract-min! wt))

               ((eq? op 'add)           ; arg is list of nodes
                ;; every time a succesful expansion happens, add is called 
                ;; (even if with empty list).
                (set! expansions (1+ expansions)) ; keep track of amount of work
                ;; update the wt-tree
                (for-each (lambda (n)
                            (set-search-node-id! n node-id)
                            (wt-tree/add! wt (cons (search-node-cost n) node-id) n)
                            (set! node-id (1+ node-id)))
                          arg))

               ((eq? op 'count)         ; no arg
                ;; could keep count as we go, but we don't do this very often
                (wt-tree/size wt))

               ((eq? op 'summary)       ; no arg
                (display* " Length of queue= " (wt-tree/size wt)
                          " Cost of first node= "
                          (if (> (wt-tree/size wt) 0)
                              (search-node-cost (wt-tree/min-datum wt))
                              #f)
                          " Expansions = " expansions
                          " Nodes added = " node-id
                          ))

               (else
                (error "Unknown operation for:" op args))))
            )))
    (fun 'add initial)
    fun
    ))

