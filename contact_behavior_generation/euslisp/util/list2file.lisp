
(defun log-surfix
  nil
  (let ((ret
	 (map string
	      #'(lambda (a) (if (= a #\ ) #\_ a))
	      (unix:asctime (unix:localtime)))))
    (subseq ret 0 (- (length ret) 1))))

(defun format-for-string
  (str pattern &rest args)
  (labels ((string-wrapper
            (d) (if (stringp d) (format nil "\"~A\"" d) d)))
    (apply
     #'format
     (append (list str pattern)
             (mapcar
              #'(lambda (d)
                  (cond
                   ((listp d) (mapcar #'string-wrapper d))
                   (t (string-wrapper d))))
              args)))))

(defun list2file
  (list file &key (depth 0) (close? t))
  (let ( (out (if (stringp file) (open file :direction :output) file)) )
    (labels ( (itter
	       (list &optional (depth depth))
	       (cond
		((null list)
		 (if (plusp depth)
		     (format-for-string out "(@ret ~A)~%" (- depth 1))))
		((not (listp list))
		 (format-for-string out "(@atm ~A)~%" list))
		((listp (car list))
		 (format-for-string out "(@dig ~A)~%" depth)
		 (itter (car list) (+ depth 1))
		 (itter (cdr list) depth))
		(t
		 (format-for-string out "(@val ~A)~%" (car list))
		 (itter (cdr list) depth)))) )
	    (itter list))
    (if close? (close out) out)))

(defun list2file-sequence
  (list out &optional (depth 0))
  (let ((out (list2file list out :depth depth :close? nil)))
    (format-for-string out "(@cut ~A)~%" depth)
    out))

(defun file2list
  (file)
  (let ( (in (if (stringp file) (open file :direction :input) file))
	 (ret nil)
	 (cut? nil) )
    (labels ((itter
	      (&optional (depth 0))
	      (let* ( (buf (read-line in nil))
		      (obj (if (and (stringp buf) (plusp (length buf)))
                               (read-from-string buf) buf)) )
					;		(print buf)
		(cond
		 ((null obj) (close in) (setq in nil))
		 ((equal (car obj) '@cut)
		  (setq cut? t)  nil)
		 ((equal (car obj) '@dig)
		  (cons (itter (+ depth 1))
			(itter depth)))
		 ((equal (car obj) '@ret) nil)
		 ((equal (car obj) '@atm) (cadr obj))
		 (t (cons (cadr obj) (itter depth)))))))
      (while (and (not cut?) in)
	(let ((buf2 (itter)))
	  (if buf2 (push buf2 ret))))
      (if cut?
	  (append (reverse ret) (file2list in))
	(reverse ret)))))
;	(cons ret (if in (file2list in) nil))))))


#|
(defun test (&optional (cnt 0)) (print cnt) (cons cnt (test (+ 1 cnt))))
(test)
... 1294 Segmentation fault
