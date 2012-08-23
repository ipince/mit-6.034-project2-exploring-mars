;;;; -*- mode:Scheme -*- ;;;;

;;; SEARCH-UPDATE (for 6.034 by TLP@mit.edu)
;;; The complete version of search-update, handles inconsistent heuristics and
;;; detects duplicates added to pending.

(declare (usual-integrations))          ; Just for MIT Scheme compiler 

;;; This is the more complete version (note the call to keep-node?)
(define (search-update current neighbors pending expanded visited)
  ;; We've expanded current, so add it to expanded list.
  (if expanded (expanded 'add (list current)))
  ;; No need to keep nodes in both lists, if we have both.
  (if (and expanded visited) (visited 'remove current))
  (cond ((or expanded visited)          ; do we have expanded or visited lists?
         (let ((kept-nodes
                (filter (lambda (node)
                          ;; Check if this state has been seen before
                           (if (and expanded visited)
                              ;; General case (we have an expanded and visited
                              ;; list) - look at actual costs and estimates.
                              (keep-node? node pending expanded visited)
                              ;; The simple case - just exclude members of either list
                              (not ((or visited expanded) 'member (search-node-state node)))))
                        neighbors)))
           ;; Add the new nodes to the pending (and visited) list
           (pending 'add kept-nodes)
           (if visited (visited 'add kept-nodes))))
        (else
         ;; just add to pending.
         (pending 'add neighbors))))

;; If the heuristic is inconsistent, then it is possible to find a better path
;; to an already expanded node.  In that case, we want to remove the old node
;; from the expanded list and keep it for insertion back in pending.  If the
;; node is not in expanded but is in the visited list, that means that it is on
;; pending and we want to remove it from there, and keep the new node for
;; insertion in pending.  This last part is just to keep the size of the
;; search-queue as small as possible.

;; In general, we keep shorter paths.

;; This function returns #t or #f indicating whether the node should be added to
;; the pending (and visited lists).
(define (keep-node? node pending expanded visited)
  (let ((enode (expanded 'member (search-node-state node)))
        (vnode (visited 'member (search-node-state node)))
        (state (search-node-state node)))

    ;; remove the old node (enode or vnode) from the appropriate lists.
    (define (remove-old-node)

      ;;; your code HERE (very few lines)

      )

    (define (inform-found-better type)  ; give informative message
      (display* "Found a better " type " to "
                (if enode "expanded"  "visited") " state "
                (state-name state) " "
                (search-node-actual (or enode vnode)) "->"
                (search-node-actual node)))

    (cond 
     ((not (or enode vnode))
      ;; state is not present in either expanded or visited, so keep it.
      #t)

     ((< (search-node-actual node) (search-node-actual (or enode vnode)))
      (inform-found-better "path")

      ;; YOUR CODE HERE (very few lines)
      
      )

     (else 
      ;; old path is equally good or better, skip this node.
      #f))))

