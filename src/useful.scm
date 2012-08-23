
(define *t:silent* #f)
(define (display* . l)
  ;; Print the list of arguments
  (cond (*t:silent* #f)
        (else
         (for-each display l)
         (newline))))

(define (filter fn l)
  (cond ((null? l) '())
        ((fn (car l)) (cons (car l) (filter fn (cdr l))))
        (else (filter fn (cdr l)))))

(define rest cdr)