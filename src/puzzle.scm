;;;; -*- mode:Scheme -*- ;;;;

;;; 8-Puzzle for 6.034 (by TLP@mit.edu).

(declare (usual-integrations))

;;; !!!!!!!!!!!TO BE LOADED AFTER LOADING SEARCH.SCM!!!!!!!!!!!!!!!!

;;; Accessors and contructor for POS (we use lists so that equal? works).
(define make-pos list)
(define pos-x first)
(define pos-y second)

;;; All the positions in a board
(define *all-positions* 
  (map (lambda (x-y) (apply make-pos x-y))
       '((0 0) (0 1) (0 2) (1 0) (1 1) (1 2) (2 0) (2 1) (2 2))))

;;; BOARD
;;; A board is a list of 3 lists, each with 3 entries
;;;   ((b00 b01 b02) (b10 b11 b12) (b20 b21 b22))
;;; Entries are numbers, except for the empty tile = #f

;;; Returns new board, with x,y entry set to value.
(define (set-board-value board pos value)
  (let ((x (pos-x pos))
        (y (pos-y pos)))
    (define (set-board-y col)
      (map (lambda (entry j)
             ;; replace the yth entry with value, 
             ;; otherwise return entry
             (if (= y j) value entry))
           col
           '(0 1 2)))
    (map (lambda (col i)
           ;; replace the xth col with modified col
           (if (= i x)
               (set-board-y col)
               col))
         board
         '(0 1 2))))

(define (board-value board pos)
  (list-ref (list-ref board (pos-x pos)) (pos-y pos)))

;;; Are two positions adjacent (horizontal or vertical)
(define (adjacent? pos1 pos2)
  (let ((dx (abs (- (pos-x pos1) (pos-x pos2))))
        (dy (abs (- (pos-y pos1) (pos-y pos2)))))
    (= 1 (+ dx dy))))

;;; MOVES

;;; A list of pos (where the empty tile can move)
(define (valid-moves board from)
  (filter (lambda (to) (valid-move? board from to))
          *all-positions*))

;;; Returns boolean
(define (valid-move? board from to)
  (and (adjacent? from to)
       (let ((content-to (board-value board to))
             (content-from (board-value board from)))
         (and content-to (not content-from))
         )))

;;; Returns new board
(define (make-move board from to)
  (if (valid-move? board from to)
      (let ((content-to (board-value board to)))
        (set-board-value (set-board-value board to #f)
                         from content-to))
      (error "Invalid move" move board)))

;;; STATE
;;; A state of the game is a list: (board empty-pos)
;;; We use a list because we require that states be compared using equal?, so 
;;; that we can hash them using equal?.  If we used structures, the same
;;; state (same board) would not be equal?

(define make-state list)
(define state-board first)
(define state-empty-pos second)

(define state-name state-board)         ; used for indexing and display

;;; Making a state from a board
(define (board->state board)
  (make-state board (find-tile board #f)))

;;; Returns a list of (neighbor cost=1)...
(define (puzzle-neighbors state)
  ;; does not need a graph input
  (map (lambda (pos)
         ;; (state cost)
         (list (make-state (make-move (state-board state) (state-empty-pos state) pos)
                           pos)
               1))
       (valid-moves (state-board state) (state-empty-pos state))))

;;; Simulates an inconsistent (but admissible) heuristic.
(define (puzzle-heuristic-value-random state goal)
  ;; (random N) returns numbers in the range [0 .. N-1]
  (random (1+
           (sum-manhattan-distance (state-board state) (state-board goal)))))

;;; Uses the weaker heuristic
(define (puzzle-heuristic-value-count state goal)
  (count-misplaced-tiles  (state-board state) (state-board goal)))

;;; Uses the stronger heuristic
(define (puzzle-heuristic-value-sum state goal)
  (sum-manhattan-distance  (state-board state) (state-board goal)))

;;; Some utilities for heuristics

;;; Returns a pos where value occurs on board.
(define (find-tile board value)
  (define (loop pos-list)
    (if (null? pos-list)
        (error "Could not find the tile" board value)
        (let ((pos (first pos-list)))
          (if (equal? value (board-value board pos))
              pos
              (loop (cdr pos-list))))))
  (loop *all-positions*))

;;; Count the tiles not at their goal position.
(define (count-misplaced-tiles board goal)
  (define (loop pos-list i)
    (if (null? pos-list)
        i
        (let* ((pos (first pos-list))
               (value (board-value board pos)))
          (if (or (not value)           ; don't count empty tile
                  (equal? value (board-value goal pos)))
              (loop (cdr pos-list) i)
              (loop (cdr pos-list) (+ i 1))))))
  (loop *all-positions* 0))

;;; Distance walking along the horiz and vert segments of a grid.
(define (manhattan-distance pos1 pos2)
  (+ (abs (- (pos-x pos1) (pos-x pos2)))
     (abs (- (pos-y pos1) (pos-y pos2)))))

;;; The example in Figure 9.2 of Nilsson's AI: A New Synthesis.
(define *nilsson-start* (board->state '((2 8 3) (1 6 4) (7 #f 5))))
(define *nilsson-goal* (board->state '((1 2 3) (8 #f 4) (7 6 5))))

;;; A goal state to go along with the starting states below
(define *goal1* (board->state '((#f 1 2) (3 4 5) (6 7 8))))

;;; A set of starting states of differing complexity
(define *start1-1* (board->state '((#f 4 2) (1 5 8) (3 6 7))))
(define *start2-1* (board->state '((8 7 6) (5 4 3) (2 1 #f))))
(define *start3-1* (board->state '((4 8 1) (3 #f 2) (6 7 5))))
(define *start4-1* (board->state '((1 6 8) (3 4 2) (7 5 #f))))
(define *start5-1* (board->state '((1 2 3) (8 #f 4) (7 6 5))))

;;; Another goal state
(define *goal2* (board->state '((7 8 1) (6 #f 2) (5 4 3))))

;;; Some testing values
(define *start1-2* (board->state '((7 1 8) (6 2 #f) (5 4 3))))
(define *start2-2* (board->state '((#f 8 1) (7 2 3) (6 5 4))))

