;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "graph-sample.lisp")
(unless (boundp '*gp*)(setq *gp* (gnuplot)))

(defun graph-panel2gp-graph
  (gp
   &key
   (xlabel "x")
   (ylabel "y")
   (ratio (/ (* 1.0 (send gp :height)) (send gp :width)))
   (font "Times-Roman,25")
   (save? nil)
   (width 5)
   )
  (gp-simple-draw
   :name (send gp :name)
   :xrange (coerce (send gp :x-range) cons)
   :yrange (coerce (send gp :y-range) cons)
   :xlabel xlabel
   :ylabel ylabel
   :ratio ratio
   :font font
   :save? save?
   :width width
   :title (send-all (send gp :data) :name)
   :data-list
   (mapcar #'(lambda (dlist)
	       (mapcar #'(lambda (vec) (coerce vec cons))
		       (send dlist :data)))
	   (send gp :data))
   :color
   (mapcar
    #'(lambda (col)
	(format nil "#~X" col))
    (send-all (send gp :data) :color))))

(defun gp-simple-draw
  (&key
   (xrange '(0 100))
   (yrange '(0 1))
   (xlabel "x")
   (ylabel "y")
   (ratio 1)
   (font "Times-Roman,25")
   (title (list "hoge"))
   (title-tmp-name title)
   (data-list (list '(0 1 2 3 4 5)))
   (step 1)
   (save? nil)
   (width 10)
   (color (list "#ff0000"))
   (name "nothing")
   )
  (if (and save? (not (stringp save?))) (setq save? name))
  (if (numberp width) (setq width (make-list (length title) :initial-element 10)))
  (setq title-tmp-name
	(let ((cnt 0) buf)
	  (mapcar #'(lambda (tt) (format nil "gp-simple-draw-tmp-file~A" (incf cnt)))
		  title-tmp-name)))
  (if save?
      (progn
	(send *gp* :command
	      "set terminal postscript eps enhanced color solid")
	(send *gp* :command
	      (format nil "set output \'~A\'" save?))))
  (send *gp* :command (format nil "set grid"))
  (send *gp* :command (format nil "set title \'~A\'" name))
  (setq data-list
	(mapcar
	 #'(lambda (dlist)
	     (let ((index -1))
	       (mapcar #'(lambda (val)
			   (incf index)
			   (if (numberp val)
			       (list index val)
			     val))
		       dlist)))
	 data-list))
;  (send *gp* :command
;	(format nil "set size ~A,~A" (car ratio) (cadr ratio)))
  (send *gp* :command
	(format nil "set size ratio ~A" ratio))
  ;;(send *gp* :command "set grid")
  (send *gp* :command "set zeroaxis")
  (send *gp* :command
	(format nil "set xrange [~A:~A]" (car xrange) (cadr xrange)))
  (send *gp* :command
	(format nil "set yrange [~A:~A]" (car yrange) (cadr yrange)))
  (send *gp* :command
	(format nil "set xlabel \"~A\" 0,0 font \"~A\""
		xlabel
		font))
  (send *gp* :command
	(format nil "set ylabel \"~A\" 0,0 font \"~A\""
		ylabel
		font))
  ;; (send *gp* :command (format nil "set key outside"))
  ;; (send *gp* :command (format nil "set key spacing 0.8"))
  (mapcar
   #'(lambda (tmp title dlist)
       (let ((out (open tmp :direction :output)))
	 (mapcar
	  #'(lambda (data)
	      (format out
		      "~A ~A~%" (car data) (cadr data)))
	  dlist)
	 (close out)))
   title-tmp-name title data-list)
  (send *gp* :command
	(apply
	 #'concatenate
	 (cons
	  string
	  (cons
	   "plot "
	   (butlast
	    (flatten
	     (mapcar
	      #'(lambda (tmp title color width)
		  (list
		   (format nil "\"~A\" title '~A' with lines linecolor rgbcolor \"~A\" lw ~A "
			   tmp title color width)
		   ", "))
	      title-tmp-name title color width)))))))
  ;; title
  ;; (mapcar
  ;;  #'(lambda (tmp title dlist)
  ;;      (let ((out (open (format nil "~A.title" tmp) :direction :output)))
  ;; 	 (format out
  ;; 		 "~A ~A~%" (caar dlist) (cadar dlist))
  ;; 	 (close out)))
  ;;  title-tmp-name title data-list)
  ;; (send *gp* :command
  ;; 	(apply
  ;; 	 #'concatenate
  ;; 	 (cons
  ;; 	  string
  ;; 	  (cons
  ;; 	   "plot "
  ;; 	   (butlast
  ;; 	    (flatten
  ;; 	     (mapcar
  ;; 	      #'(lambda (tmp title color width)
  ;; 		  (list
  ;; 		   (format nil "\"~A\"  title '~A' with lines linecolor rgbcolor \"~A\" lw ~A"
  ;; 			   (format nil "~A.title" tmp) title color width)
  ;; 		   ", "))
  ;; 	      title-tmp-name title color width)))))))
  ;;
  (if save?
      (progn
	(send *gp* :command "set terminal x11")
	(send *gp* :command "replot")))
  (unix:sleep 1)
  (mapcar
   #'(lambda (tmp title)
       (unix:system
	(format nil "rm -rf ~A" tmp)))
   title-tmp-name title)
  )
