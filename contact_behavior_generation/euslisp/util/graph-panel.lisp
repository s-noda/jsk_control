(require "graph-data.lisp")

(defclass graph-panel
  :super x::panel
  :slots (max-pos min-pos
	  origin
	  margin
	  data
	  buffer-image
	  name
	  mouse-pos
	  bk-image
	  )
  )

(defmethod graph-panel
  (:create
   (width height
	  &key
	  (minpos #f(-100 -100))
	  (maxpos #f(100 100))
	  (mar 25)
	  (title "graphPanel")
	  &rest args)
   (send-super* :create :width width :height height args)
   (send self :create-buffer-image width height)
   (send self :name title)
   (send self :font x::font-cour12)
   (setq origin #f(0 0))
   (setq data nil)
;   (setq data (instance graphData :init :color #x0000FF :max-length 600))
;   (send data :color #x0000FF)
;   (send data :max-length 600)
   (setq margin mar)
   (send self :set-range minpos maxpos)
   (send self :color #xFFFFFF)
   (send self :draw-axis)
   (send self :message-string "created")
   (send self :repaint)
   (send self :set-event-proc
	 :buttonrelease-left :repaint self)
   self
   )
  (:name
   (&optional tit)
   (if tit
       (progn
	 (send self :title tit)
	 (setq name tit))
     name))
  (:create-buffer-image
   (width height)
   (setq buffer-image
	 (instance x::xpixmap :create :width width :height height))
   (send buffer-image :background #x000000)
   (send buffer-image :clear))
  (:buffer-image
   (&optional image)
   (if image (setq buffer-image image) buffer-image))
  (:bk-image
   (&optional image)
   (if image (setq bk-image image) bk-image))
  (:fit-range
   (&optional (margin 0.1))
   (let* ((data (flatten (send-all (send self :data) :data)))
	  (xsort (sort (mapcar #'copy-seq data)
		       #'(lambda (a b) (<= (aref a 0) (aref b 0)))))
	  (ysort (sort (mapcar #'copy-seq data)
		       #'(lambda (a b) (<= (aref a 1) (aref b 1)))))
	  (xmar (* margin (- (aref (car (last xsort)) 0)
			     (aref (car xsort) 0))))
	  (ymar (* margin (- (aref (car (last ysort)) 1)
			     (aref (car ysort) 1)))))
     (list (float-vector (- (aref (car xsort) 0) xmar)
			 (- (aref (car ysort) 1) ymar))
	   (float-vector (+ (aref (car (last xsort)) 0) xmar)
			 (+ (aref (car (last ysort)) 1) ymar)))))
  (:fit-draw
   (&key (margin 0.1) (line-graph? t))
   (apply #'send (append (list self :set-range)
			 (send self :fit-range margin)))
   (if line-graph?
       (send self :simple-draw-with-line)
     (send self :simple-draw))
   self)
  (:repaint
   nil
   (case (send (class buffer-image) :name)
     ('x:xpixmap
      (send self :copy-from buffer-image)
      (send self :flush))
     (t ;; unused
      (send-super :putimage (send buffer-image :to24))
      (send self :flush))))
  (:forcus-pos
   (&optional pos)
   (if pos
       (setq origin (v- #f(0 0) pos))
     (v- #f(0 0) origin)))
  (:set-range
   (minpos maxpos)
   (let* ( (diff (scale 0.5 (v- maxpos minpos)))
	   (center (scale 0.5 (v+ maxpos minpos)))
	   (min (v- #f(0 0) diff))
	   (max (v+ #f(0 0) diff)) )
     (setq min-pos min)
     (setq max-pos max)
     (send self :forcus-pos center)
;     (send self :draw-axis))
   ))
  (:min-pos-screen nil (float-vector margin
				     (- (send self :height) margin)))
  (:max-pos-screen nil (float-vector (- (send self :width) margin)
				     margin))
  (:screen-pos
   (pos)
   (let* ( (graph-vec (v- (v+ pos origin) min-pos))
	   (graph-scale-vec (v- max-pos min-pos))
	   (screen-min-vec (send self :min-pos-screen))
	   (screen-scale-vec (v- (send self :max-pos-screen)
				 (send self :min-pos-screen)))
	   (scale-vec
	    (map float-vector
		 #'(lambda (gscale sscale)
		     (/ sscale (if (zerop gscale) 1 gscale)))
		 graph-scale-vec screen-scale-vec))
	   (screen-vec
	    (map float-vector
		 #'*
		 graph-vec scale-vec)) )
;     (print scale-vec) (print screen-vec)
     (map float-vector
	  #'(lambda (v)
	      (case v
		(*nan* 0)
		(*inf* 0)
		(t v)))
	  (v+ screen-min-vec screen-vec))
     )
   )
  (:x-min nil
	  (let* ( (vec (v+ (float-vector (aref min-pos 0) 0)
			   (float-vector 0 (aref origin 1))))
		  (y-vec (aref vec 1))
		  (y-max (aref max-pos 1))
		  (y-min (aref min-pos 1)) )
	    (cond
	     ((> y-vec y-max) (setf (aref vec 1) y-max))
	     ((< y-vec y-min) (setf (aref vec 1) y-min)))
	    (v- vec origin)))
  (:x-max nil
	  (let* ( (vec (v+ (float-vector (aref max-pos 0) 0)
			   (float-vector 0 (aref origin 1))))
		  (y-vec (aref vec 1))
		  (y-max (aref max-pos 1))
		  (y-min (aref min-pos 1)) )
	    (cond
	     ((> y-vec y-max) (setf (aref vec 1) y-max))
	     ((< y-vec y-min) (setf (aref vec 1) y-min)))
	    (v- vec origin)))
  (:y-min nil
	  (let* ( (vec (v+ (float-vector 0 (aref min-pos 1))
			   (float-vector (aref origin 0) 0)))
		  (x-vec (aref vec 0))
		  (x-max (aref max-pos 0))
		  (x-min (aref min-pos 0)) )
	    (cond
	     ((> x-vec x-max) (setf (aref vec 0) x-max))
	     ((< x-vec x-min) (setf (aref vec 0) x-min)))
	    (v- vec origin)))
  (:y-max nil
	  (let* ( (vec (v+ (float-vector 0 (aref max-pos 1))
			   (float-vector (aref origin 0) 0)))
		  (x-vec (aref vec 0))
		  (x-max (aref max-pos 0))
		  (x-min (aref min-pos 0)) )
	    (cond
	     ((> x-vec x-max) (setf (aref vec 0) x-max))
	     ((< x-vec x-min) (setf (aref vec 0) x-min)))
	    (v- vec origin)))
  (:x-range nil (float-vector (aref (send self :x-min) 0)
			      (aref (send self :x-max) 0)))
  (:y-range nil (float-vector (aref (send self :y-min) 1)
			      (aref (send self :y-max) 1)))
  (:data (&optional dat)
	 (if dat (setq data (append data (list dat))) data))
  (:set-data (dat) (setq data dat))
;  (:set-color
;   (col)
;   (send self :color color))
  (:font-size
   (str)
   (x::textdots str (send self :font)))
  (:message-string
   (str &optional (pos :left-down))
   (let ((size (x::textdots str)))
     (case pos
       (:left-down
	(send self :image-string
	      (floor (* 0.01 (send self :width)))
	      (floor (* 0.99 (send self :height))) str))
       (:left-top
	(send self :image-string
	      0
	      (floor (* 1.01 4 (aref size 1)))
	      str))
       (:right-top
	(send self :image-string
	      (- (send self :width) (aref size 2))
	      (* 4 (aref size 1)) str))
       (:right-down
	(send self :image-string
	      (floor (* 0.99 (- (send self :width) (aref size 2))))
	      (floor (* 0.99 (send self :height))) str))
       (t (apply #'send (append (list self :image-string)
				pos
				(list str)))))))
  (:string-screen
   (pos str &optional (center #f(0 0)))
   (let ( (pos-screen (v+ center (send self :screen-pos pos))) )
;     (print pos-screen)
     (send self :image-string
	   (round (aref pos-screen 0))
	   (round (aref pos-screen 1))
	   str)
     ))
  (:plot
   (pos &optional (size 3))
   (send self :fill-rectangle
	 (round (- (aref pos 0) (/ size 2.0)))
	 (round (- (aref pos 1) (/ size 2.0)))
	 size size))
  (:plot-screen
   (pos &optional (size 3))
   (send self :plot (send self :screen-pos pos) size))
  (:line-screen
   (p1 p2 &optional (size 3))
   (let ((s1 (send self :screen-pos p1))
	 (s2 (send self :screen-pos p2)))
     (send self :plot s1 size)
     (send self :plot s2 size)
     (send self :draw-line s1 s2)))

 ;; @Override
  (:background
   (col)
   (if buffer-image
       (send buffer-image :background col)
     (send-super :background col)))
  (:color
   (color)
   (if buffer-image
       (send buffer-image :color color)
     (send-super :color color)))
  (:image-string
   (x y str)
   (cond
;    ((or (eq (abs x) (abs *nan*))
;	 (eq (abs x) *inf*)
;	 (eq (abs y) (abs *nan*))
;	 (eq (abs y) *inf*))
 ;    (print 'invalid-pos))
    (buffer-image
     (send buffer-image :image-string x y str))
    (t (send-super :image-string x y str))))
  (:fill-rectangle
   (x y width height)
   (if buffer-image
       (send buffer-image :fill-rectangle x y width height)
     (send-super :fill-rectangle x y width height)))
  (:draw-circul
   (x y width height)
   (if buffer-image
       (send buffer-image :arc
	     x y width height (deg2rad 0) (deg2rad 360))
     (send-super :arc
		 x y width height (deg2rad 0) (deg2rad 360))))
  (:resize
   (width height &rest args)
   (send self :create-buffer-image width height)
   (send-super* :resize width height args)
;   (send self :repaint)
   )
  (:draw-line
   (pos1 pos2)
   (if buffer-image
       (send buffer-image :draw-line pos1 pos2)
     (send-super :draw-line pos1 pos2)))
  (:clear nil
	  (cond
	   (bk-image
	    (send self :putimage (send bk-image :to24))
	    (send buffer-image :copy-from self))
	   (buffer-image
	    (send buffer-image :clear))
	   (t (send-super :clear))))
  (:width nil
	  (if buffer-image
	      (send buffer-image :width)
	    (send-super :width)))
  (:height nil
	   (if buffer-image
	       (send buffer-image :height)
	     (send-super :width)))
  ;; --------------------------------------------

  (:plot-data
   (&optional (size 3))
   (dolist (d data)
;     (send self :color #x000000)
;     (send self :draw-axis)
     (send self :color (send d :color))
     (dolist (dat (send d :range-filter
			(v- min-pos origin)
			(v- max-pos origin)))
       (send self :plot-screen dat size))))
  (:plot-with-line-data
   (&optional (size 3))
   (dolist (d data)
;     (send self :color #x000000)
;     (send self :draw-axis)
     (send self :color (send d :color))
     (let ((pre-pos))
       (dolist (dat (send d :range-filter
			  (v- min-pos origin)
			  (v- max-pos origin)))
	 (if pre-pos
	     (send self :draw-line
		   (send self :screen-pos pre-pos)
		   (send self :screen-pos dat)))
	 (setq pre-pos dat)))))
;     (send self :repaint)))
  (:name-index
   (name)
   (labels ( (name-index
	      (name list num)
	      (cond
	       ((null list)  nil)
	       ((equal name (send (car list) :name)) num)
	       (t (name-index name (cdr list) (+ num 1))))) )
     (name-index name data 0)))
  (:plot-add
   (pos &optional (index 0))
;   (send self :forcus-pos pos)
   (send (nth index data) :add pos))
  (:plot-name-add
   (pos &optional (name "nothing"))
;   (send self :forcus-pos pos)
   (send (nth (send self :name-index name) data) :add pos))
;   (send self :plot-data))
  (:data-delete-index
   (index)
   (send (nth index data) :data-delete))
  (:data-delete
   nil
   (send-all data :data-delete))
  (:data-remove
   nil
   (setq data nil))
  (:draw-measure-string
   (&key (x-min "") (x-max "") (y-min "") (y-max ""))
   (progn
     (let ( (str (format nil "~A" (aref (send self :x-min) 0))) )
       (send self :string-screen (send self :x-min) str)
       (send self :string-screen
	     (send self :x-min)
	     x-min
	     (float-vector 0 (* -10 (aref (send self :font-size str) 1)))))
     (let ( (str (format nil "~A" (aref (send self :x-max) 0))) )
       (send self :string-screen
	     (send self :x-max)
	     str
	     (float-vector (* -1 (aref (send self :font-size str) 2)) 0))
       (send self :string-screen
	     (send self :x-max)
	     x-max
	     (float-vector (* -1 (aref (send self :font-size x-max) 2))
			   (* -10 (aref (send self :font-size str) 1)))))
     (let ( (str (format nil "~A" (aref (send self :y-min) 1))) )
       (send self :string-screen
	     (send self :y-min)
	     (format nil "~A~A" (aref (send self :y-min) 1) y-min))
       (send self :string-screen
	     (send self :y-min)
	     y-min
	     (float-vector 0 (* 10 (aref (send self :font-size str) 1)))))
     (let ( (str (format nil "~A" (aref (send self :y-max) 1))) )
       (send self :string-screen
	     (send self :y-max)
	     str
	     (float-vector 0 (* 1 (aref (send self :font-size str) 1)))
	     )
       (send self :string-screen
	     (send self :y-max)
	     y-max
	     (v+
	      (float-vector 0 (* 1 (aref (send self :font-size str) 1)))
	      (float-vector 0 (* 10 (aref (send self :font-size str) 1))))))
     ))
  (:draw-label
   nil
   (if (null data)
       nil
     (let* ( (line 0)
	     (name-list (send-all data :name))
	     (name-font-list (mapcar
			      #'(lambda (name) (send self :font-size name))
			      name-list))
	     (name-max-len (apply #'vmax name-font-list))
	     (str-x (- (send self :width) (aref name-max-len 2))) )
       (dolist (dat data)
	 (setq line (+ 1 line))
	 (let ( (height
		 (* 4 line (aref name-max-len 1))) )
	   (send self :color (send dat :color))
	   (send self :image-string str-x height (send dat :name))))))
   )
  (:draw-color-label
   (&key
    (step 0.5)
    (width (* (send self :width) 0.03))
    (height (* (send self :height) 0.7))
    (x (- (send self :width) (* 1.5 width)))
    (y (/ (- (send self :height) height) 4.0)))
   (let ((now-y y))
     (mapcar
      #'(lambda (dat)
	  (send self :color (send dat :color))
	  (mapcar
	   #'(lambda (hoge)
	       (send self :draw-line
		     (integer-vector (round x) (round now-y))
		     (integer-vector (round (+ x width)) (round now-y)))
	       (setq now-y (+ now-y step)))
	   (make-list
	    (let ((buf (round (/ height (length data)))))
	      (if (zerop buf) 1 buf)))))
      data)
     (send self :color #xFFFFFF)
     (let ((font (send self :font-size (send (car data) :name))))
       (send self :image-string
	     (min (round (- (send self :width) (* 1.2 (aref font 2)))) (round x))
	     (round (- y (* 4 (aref font 1))))
	     (send (car data) :name)))
     (let ((font (send self :font-size (send (car (last data)) :name))))
       (send self :image-string
	     (min (round (- (send self :width) (* 1.2 (aref font 2)))) (round x))
	     (round (+ (+ y height) (* 4 (aref font 1))))
	     (send (car (last data)) :name)))
     ))
  (:draw-axis
   (&key (draw-measure-string t)
	 (measure-range "~A")
	 (octave 20)
	 (str-step 5)
	 (y-offset 0))
   (progn
     (send self :draw-line
	   (send self :screen-pos (send self :y-min))
	   (send self :screen-pos (send self :y-max)))
     (send self :draw-line
	   (send self :screen-pos (send self :x-min))
	   (send self :screen-pos (send self :x-max)))
     (labels ( (draw-measure
		(octave str-step edge-x edge-y)
		(let* ( (x-start (aref (send self :x-min) 0))
			(y-start (aref (send self :y-min) 1))
			(x-step (/ (- (aref (send self :x-max) 0) x-start)
				      octave))
			(y-step (/ (- (aref (send self :y-max) 1) y-start)
				   octave)) )
		  (dotimes (i (+ octave 1))
		    (send self :draw-line
 			  (send self :screen-pos
				(v+ (float-vector (* i x-step) 0)
				    (send self :x-min)))
			  (send self :screen-pos
				(v+ (float-vector (* i x-step) edge-y)
				    (send self :x-min))))
		    (send self :draw-line
			  (send self :screen-pos
				(v+ (float-vector 0 (* i y-step))
				    (send self :y-min)))
			  (send self :screen-pos
				(v+ (float-vector edge-x (* i y-step))
				    (send self :y-min))))
		    (if (zerop (mod i str-step))
			(progn
			  (send self :string-screen
				(v+ (float-vector (* (- i 0.5) x-step) y-offset)
				    (send self :x-min))
				(format nil measure-range (+ x-start (* i x-step))))
			  (send self :string-screen
				(v+ (float-vector 0 (* i y-step))
				    (send self :y-min))
				(format nil measure-range (+ y-start (* i y-step))))
			  ))
		    )
		  )) )
       (let ( (edge (scale (/ -1 50.0) (v- max-pos min-pos))) )
	 (draw-measure octave str-step (aref edge 0) (aref edge 1) )))
;     (if draw-measure-string (send self :draw-measure-string))))
     ))

  (:simple-draw
   nil
   (send self :clear)
   (send self :color #xFFFFFF)
   (send self :draw-axis)
   (send self :plot-data)
   (send self :draw-label)
   (send self :message-string "plot")
   (send self :repaint)
   self)

  (:simple-draw-with-line
   (&optional (draw? t))
   (send self :clear)
   (send self :color #xFFFFFF)
   (send self :draw-axis)
   (send self :plot-with-line-data)
   (send self :draw-label)
   (send self :message-string "plot")
   (if draw? (send self :repaint))
   self)

;; ------------- key event ---------------
  )

(defvar *demo-graph*)
(defun demo-graph-test
  (&optional
   (g (cond
       ((null *demo-graph*)
	(setq *demo-graph* (instance graph-panel :create 640 480))
	(send *demo-graph* :set-range #f(-5 0) #f(5 2))
	(let ( (data1 (instance graph-data :init))
	       (data2 (instance graph-data :init)) )
	  (send data1 :color #x00FF00)
	  (send data1 :name "cos")
	  (send data1 :max-length 5000)
	  (send data2 :color #x0000FF)
	  (send data2 :name "sin")
	  (send data2 :max-length 5000)
	  (send *demo-graph* :data data1)
	  (send *demo-graph* :data data2))
	*demo-graph*)
       (t *demo-graph*))))
  (let ( (time 0) )
    (do-until-key
     (unix:usleep 100)
     (send g :clear)
     ;;
     (send g :plot-add (float-vector time (+ 1 (* 1 (cos time)))) 0)
     (send g :plot-add (float-vector time (+ 1 (* 1 (sin time)))) 1)
     (send g :forcus-pos (float-vector time 1))
     ;;
     (send g :color #xFFFFFF)
     (send g :draw-axis)
     ;;
     (send g :plot-data)
     ;;
     (send g :draw-label)
     (send g :message-string "right-down" :right-down)
     (send g :message-string "left-top" :left-top)
     (send g :message-string "left-down" :left-down)
     (send g :message-string "right-top" :right-top)
     (send g :repaint)
     (setq time (+ time 0.1)))))