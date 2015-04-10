(require "graph-panel.l")

(defvar *graph-sample* nil)
(defvar *graph-mutex* (sys:make-mutex-lock))

(defun quit-graph
  nil
  (send-all *graph-sample* :quit)
  (setq *graph-sample* nil))

(defun create-graph
  (&optional
   (graph-name "sampleGraph")
   &key
   (size '(500 500))
   (name-list '("a" "b" "c"))
   (range (list #f(-100 0) #f(100 100)))
   (inc-data-list nil)
   (data-list (if inc-data-list
		  (mapcar #'(lambda (dl)
			      (let ((index -1))
				(map cons #'(lambda (d) (float-vector (incf index) d)) dl)))
			  inc-data-list)
		'(nil nil nil)))
   (color-list
    (let* ( (count -1)
	    (offset (/ 180.0
		       (- (length name-list) 1))) )
      (mapcar
       #'(lambda (name)
	   (setq count (+ count 1))
	   (round
	    (apply #'+
		   (map cons
			#'*
			(reverse (vector #x000001 #x000100 #x010000))
			(hls2rgb (* offset count) 0.5 0.5)))))
       name-list)))
   (max-length 500)
   )
  ;; ---------------- graph ----------------------
  (let (graph)
    (setq graph (instance graph-panel :create (car size) (cadr size)
				   :title graph-name))
    (send graph :set-range (car range) (cadr range))
    (send graph :clear)
    (send graph :color #x000000)
    (send graph :draw-axis)
    (send graph :message-string "stand by")
    (send graph :repaint)
    ;;
    (labels ( (set-data
	       nil
	       (let* ( (databuf data-list) (color-buf color-list) )
		 (dolist (name name-list)
		   (let ( (data (instance graph-data
					  :init :data (car databuf))) )
		     (setq databuf (cdr databuf))
		     (send data :max-length max-length)
		     (send data :color (car color-buf))
		     (setq color-buf (cdr color-buf))
		     (send data :name name)
		     (send graph :data data))))) )
	    (set-data))
    (send graph :clear)
    (send graph :color #x000000)
    (send graph :draw-axis)
    (send graph :draw-label)
    (send graph :message-string "ready")
    (send graph :repaint)
    (push graph *graph-sample*)
    graph
    ))

(defun add-data-to-graph
  (data-list &key
	     (title-change? t)
	     (draw? t) (graph (car *graph-sample*)) (time-step 1)
	     (pre-pos nil) (forcus-flag nil))
  (sys:mutex-lock *graph-mutex*)
  (if (find-method graph :data)
      (let* ( (count 0)
	      (pre-pos
	       (or
		pre-pos
		(let ( (recent-pos
			(car (send
			      (car (send graph :data))
			      :data))) )
		  (if recent-pos
		      recent-pos
		    (send graph :forcus-pos)))))
	      (forcus (v+ (float-vector time-step 0)
			  (float-vector
			   (aref pre-pos 0)
			   (aref (send graph :forcus-pos) 1))))
	      (data-class (send graph :data))
	      (forcus-flag (or forcus-flag (not (vectorp (car data-list)))))
	      (data-list
	       (mapcar #'(lambda (a)
			   (if (vectorp a)
			       a
			     (float-vector (aref forcus 0) a)))
		       data-list))
	      (data-class-sort-x
	       (sort
		(mapcar #'(lambda (a b) (cons (aref a 0) b))
			data-list data-class)
		#'(lambda (a b) (<= (car a) (car b)))))
	      (data-class-sort-y
	       (sort
		(mapcar #'(lambda (a b) (cons (aref a 1) b))
			data-list data-class)
		#'(lambda (a b) (<= (car a) (car b)))))
	      )
	(dolist (data data-list)
	  (if title-change?
	      (send (nth count data-class) :name
		    (format nil "[~f ~f]=~A"
			    (aref data 0) (aref data 1)
			    (or
			     (nth 1
				  (reg-match "^\\[.*?\\]=(.*)$"
					     (send (nth count data-class) :name)))
			     (send (nth count data-class) :name)))))
	  (send graph :plot-add data count)
	  (setq count (+ 1 count)))
	(if forcus-flag
	    (send graph :forcus-pos forcus))
	(if draw?
	    (progn
	      (send graph :clear)
	      (send graph :color #xFFFFFF)
	      (send graph :draw-axis)
	      (send graph :plot-data)
	      (send graph :draw-label)
	      (send graph :message-string
		    (format nil "~A ~A < data < ~A ~A"
			    (caar data-class-sort-x)
			    (caar data-class-sort-y)
			    (caar (last data-class-sort-x))
			    (caar (last data-class-sort-y))
			    ))
	      (send graph :repaint))
	  )))
	(sys:mutex-unlock *graph-mutex*)
	)

(defun place-graph-in-order
  (&key
   (graph *graph-sample*)
   (size (list (send (car graph) :width)
	       (send (car graph) :height)))
   (root-size (list (- (send x::*root* :width) (car size))
		    (- (send x::*root* :height) (cadr size))))
   (x 0)
   (y 0))
  (if (null graph)
      nil
    (let ((g (car graph)))
      (send g :move x y)
      (send g :repaint)
      (setq x (+ x (car size)))
      (if (> x (car root-size))
	  (progn
	    (setq x 0)
	    (setq y (+ y (cadr size)))))
      (if (> y (cadr root-size))
	  (setq y 0))
      (place-graph-in-order
       :graph (cdr graph)
       :size size
       :root-size root-size
       :x x
       :y y))))