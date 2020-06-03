;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :ropha-demo)

(defparameter *object-cad-models*
  '(;; (:cup . "cup_eco_orange")
    ;; (:bowl . "edeka_red_bowl")
    ))

(defparameter *object-colors*
  '((:spoon . "blue")))

;; (defparameter *object-spawning-poses*
;;   '((:bowl . ((1.45 1.05 0.48) (0 0 0 1))) ; left-middle-drawer
;;     (:cup . ((1.42 0.7 0.48) (0 0 0 1))) ; left-middle-drawer
;;     (:knife . ((1.43 0.9 0.74132) (0 0 0 1))) ;; left-upper-drawer
;;     ;; So far only this orientation works
;;     (:breakfast-cereal . ((1.412 1.490 1.2558) (0 0 -0.55557d0 0.83147d0)))
;;     ;; ((:breakfast-cereal . ((1.398 1.490 1.2558) (0 0 0.7071 0.7071)))
;;     ;; (:breakfast-cereal . ((1.1 1.49 1.25) (0 0 0.7071 0.7071)))
;;     (:plate . ((-0.75 1.7 0.95) (0 0 0.7071 0.7071)))))
(defparameter *object-spawning-poses*
  '("sink_area_surface"
    ((:breakfast-cereal . ((0.2 -0.15 0.1) (0 0 0 1)))
     (:cup . ((0.2 -0.35 0.1) (0 0 0 1)))
     (:bowl . ((0.18 -0.55 0.1) (0 0 0 1)))
     (:knife . ((0.15 -0.4 -0.05) (0 0 0 1)))
     (:plate . ((2.47 -0.55 0.2) (0 0 0 1)))))
  "Relative poses on sink area")

(defparameter *object-placing-poses*
  '((:breakfast-cereal . ((-0.78 0.9 0.95) (0 0 1 0)))
    (:cup . ((-0.79 1.35 0.9) (0 0 0.7071 0.7071)))
    (:bowl . ((-0.76 1.19 0.88) (0 0 0.7071 0.7071)))
    (:knife . ((-0.78 1.5 0.86) (0 0 0 1)))
    (:plate . ((-0.75 1.7 0.95) (0 0 0.7071 0.7071))))
  "Absolute poses on kitchen_island.")

(defparameter *object-grasping-arms*
  '(;; (:breakfast-cereal . :right)
    ;; (:cup . :left)
    ;; (:bowl . :right)
    ;; (:spoon . :right)
    ;; (:milk . :right)
    ))


(defparameter *sink-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 0.70d0 0.0)
   (cl-transforms:make-identity-rotation)))
(defparameter *island-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -0.2d0 1.5d0 0.0)
   (cl-transforms:make-quaternion 0 0 1 0)))
(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))
(defmethod exe:generic-perform :before (designator)
  (roslisp:ros-info (demo perform) "~%~A~%~%" designator))

(cpl:def-cram-function park-robot ()
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (exe:perform
       (desig:an action
                 (type positioning-arm)
                 (left-configuration park)
                 (right-configuration park)))
      ;; (let ((?pose (cl-transforms-stamped:make-pose-stamped
      ;;               cram-tf:*fixed-frame*
      ;;               0.0
      ;;               (cl-transforms:make-identity-vector)
      ;;               (cl-transforms:make-identity-rotation))))
      ;;   (exe:perform
      ;;    (desig:an action
      ;;              (type going)
      ;;              (target (desig:a location
      ;;                               (pose ?pose))))))
      (exe:perform (desig:an action (type opening-gripper) (gripper (left right))))
      ;; (exe:perform (desig:an action (type looking) (direction forward)))
      )))



;; (defun initialize ()
;;   (sb-ext:gc :full t)
;;   ;;(when ccl::*is-logging-enabled*
;;   ;;    (setf ccl::*is-client-connected* nil)
;;   ;;    (ccl::connect-to-cloud-logger)
;;   ;;    (ccl::reset-logged-owl))

;;   ;; (setf proj-reasoning::*projection-checks-enabled* t)

;;   (btr:detach-all-objects (btr:get-robot-object))
;;   (btr:detach-all-objects (btr:get-environment-object))
;;   (btr-utils:kill-all-objects)
;;   (setf (btr:joint-state (btr:get-environment-object)
;;                          "sink_area_left_upper_drawer_main_joint")
;;         0.0
;;         (btr:joint-state (btr:get-environment-object)
;;                          "sink_area_left_middle_drawer_main_joint")
;;         0.0
;;         (btr:joint-state (btr:get-environment-object)
;;                          "iai_fridge_door_joint")
;;         0.0
;;         (btr:joint-state (btr:get-environment-object)
;;                          "oven_area_area_right_drawer_main_joint")
;;         0.0
;;         (btr:joint-state (btr:get-environment-object)
;;                          "sink_area_trash_drawer_main_joint")
;;         0)
;;   (btr-belief::publish-environment-joint-state
;;    (btr:joint-states (btr:get-environment-object)))

;;   (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

;;   ;; (coe:clear-belief)

;;   (btr:clear-costmap-vis-object)

;;   ;; (setf cram-robot-pose-guassian-costmap::*orientation-samples* 3)
;; )


;; (defun make-poses-relative (spawning-poses)
;;   "Gets an associative list in a form of (FRAME ((TYPE . COORDINATES-LIST) ...)),
;; where coordinates-list is defined in the FRAME coordinate frame.
;; Converts these coordinates into CRAM-TF:*FIXED-FRAME* frame and returns a list in form
;;  ((TYPE . POSE) ...)."
;;   (when spawning-poses
;;     (let* ((map-T-surface (cl-transforms:pose->transform
;;                            (btr:link-pose (btr:get-environment-object)
;;                                           (first spawning-poses)))))
;;       (mapcar (lambda (type-and-pose-list)
;;                 (destructuring-bind (type . pose-list)
;;                     type-and-pose-list
;;                   (let* ((surface-T-object
;;                            (cl-transforms:pose->transform (cram-tf:list->pose pose-list)))
;;                          (map-T-object
;;                            (cl-transforms:transform* map-T-surface surface-T-object))
;;                          (map-P-object
;;                            (cl-tf:transform->pose map-T-object)))
;;                     `(,type . ,map-P-object))))
;;               (second spawning-poses)))))


;; (defun spawn-objects-on-sink-counter (&key
;;                                         (object-types '(:breakfast-cereal
;;                                                         :cup
;;                                                         :bowl
;;                                                         :plate
;;                                                         :knife
;;                                                         ))
;;                                         (spawning-poses-relative *object-spawning-poses*)
;;                                         (random NIL))
;;   ;; make sure mesh paths are known, kill old objects and destroy all attachments
;;   (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
;;   (btr-utils:kill-all-objects)
;;   (btr:detach-all-objects (btr:get-robot-object))
;;   (btr:detach-all-objects (btr:get-environment-object))

;;   ;; spawn objects
;;   (let* ((spawning-poses-absolute
;;            (make-poses-relative spawning-poses-relative))
;;          (objects
;;            (mapcar (lambda (object-type)
;;                      (let* (;; generate object name of form :TYPE-1
;;                             (object-name
;;                               (intern (format nil "~a-1" object-type) :keyword))
;;                             ;; spawn object in Bullet World at default pose
;;                             (object
;;                               (btr-utils:spawn-object object-name object-type))
;;                             ;; calculate new pose: either random or from input list
;;                             (object-pose
;;                               (if random
;;                                   ;; generate a pose on a surface
;;                                   (let* ((aabb-z
;;                                            (cl-transforms:z
;;                                             (cl-bullet:bounding-box-dimensions
;;                                              (btr:aabb object)))))
;;                                     (cram-tf:translate-pose
;;                                      (desig:reference
;;                                       (if (eq object-type :knife)
;;                                           (desig:a location
;;                                                    (in (desig:an object
;;                                                                  (type drawer)
;;                                                                  (urdf-name
;;                                                                   sink-area-left-upper-drawer-main)
;;                                                                  (part-of kitchen)))
;;                                                    (side front)
;;                                                    (range 0.2)
;;                                                    (range-invert 0.12))
;;                                           (desig:a location
;;                                                    (on (desig:an object
;;                                                                  (type counter-top)
;;                                                                  (urdf-name sink-area-surface)
;;                                                                  (part-of kitchen)))
;;                                                    ;; below only works for knowrob sem-map
;;                                                    ;; (centered-with-padding 0.1)
;;                                                    (side left)
;;                                                    (side front))))
;;                                      :z-offset (/ aabb-z 2.0)))
;;                                   ;; take the pose from the function input list
;;                                   (cdr (assoc object-type spawning-poses-absolute))))
;;                             ;; rotate new pose randomly around Z
;;                             (rotated-object-pose
;;                               (cram-tf:rotate-pose object-pose
;;                                                    :z (/ (* 2 pi) (random 10.0)))))
;;                        ;; move object to calculated pose on surface
;;                        (btr-utils:move-object object-name rotated-object-pose)
;;                        ;; return object
;;                        object))
;;                    object-types)))

;;     ;; make sure generated poses are stable, especially important for random ones
;;     ;; TDOO: if unstable, call itself

;;     ;; stabilize world
;;     (btr:simulate btr:*current-bullet-world* 100)

;;     ;; attach spoon to the drawer
;;     (btr:attach-object (btr:object btr:*current-bullet-world* :environment)
;;                        (btr:object btr:*current-bullet-world* :knife-1)
;;                        :link "sink_area_left_upper_drawer_main")

;;     ;; return list of BTR objects
;;     objects))


(defun finalize ()
  ;; (setf pr2-proj-reasoning::*projection-reasoning-enabled* nil)

  (when ccl::*is-logging-enabled*
   (ccl::export-log-to-owl "ropha-280619-1.owl")
   ;; (ccl::export-belief-state-to-owl "ease_milestone_2018_belief.owl")
   )
  (sb-ext:gc :full t))

;; (defun opening-test ()

;;   (pr2-pms:with-real-robot
;;   ;; (urdf-proj:with-simulated-robot
;;    ;; (initialize)
;;    (park-robot)


;;    (let* ((?pose (cl-transforms-stamped:make-pose-stamped
;;                   "map" 0.0
;;                   (cl-transforms:make-3d-vector 0.55 0.4 0)
;;                   ;; (cl-transforms:make-identity-rotation)
;;                   (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (* -0.5 pi))
;;                   ))

;;           (?robot-location-designator (desig:a location (pose ?pose)))
;;           )

;;      (cpl:with-failure-handling
;;       ((cpl:plan-failure (e)
;;                          (declare (ignore e))
;;                          (return)))

;;        (cram-executive:perform
;;         (desig:an action
;;                 (type going)
;;                 (target ?robot-location-designator))))

;;      (exe:perform
;;       (let ((?handle-aproach-pose (cl-tf:make-pose-stamped
;;                     cram-tf:*fixed-frame* 0.0
;;                     ;; (cl-transforms:make-3d-vector 1.1 0.6 0.92)
;;                     (cl-transforms:make-3d-vector 1.1 0.9 0.77)
;;                     ;; (cl-transforms:make-identity-rotation)
;;                     (cl-transforms:euler->quaternion :ax (/  pi  2))
;;                     )))
;;         (desig:a motion (type moving-tcp) (left-pose ?handle-aproach-pose))))

;;      (exe:perform
;;       (let ((?handle-pose (cl-tf:make-pose-stamped
;;                     cram-tf:*fixed-frame* 0.0
;;                     (cl-transforms:make-3d-vector 1.18 0.9 0.77)
;;                     ;; (cl-transforms:make-3d-vector 1.27 0.6 0.92)
;;                     ;; (cl-transforms:make-identity-rotation)
;;                     (cl-transforms:euler->quaternion :ax (/  pi  2))
;;                     )))
;;         (desig:a motion (type moving-tcp) (left-pose ?handle-pose))))

;;      (exe:perform
;;       (desig:an action
;;              (type setting-gripper)
;;              (gripper left)
;;              (position 0)))

;;      (exe:perform
;;       (let ((?handle-pose (cl-tf:make-pose-stamped
;;                     cram-tf:*fixed-frame* 0.0
;;                     (cl-transforms:make-3d-vector 1.1 0.9 0.76)
;;                     ;; (cl-transforms:make-3d-vector 1.25 0.6 1.12)
;;                     ;; (cl-transforms:make-identity-rotation)
;;                     (cl-transforms:euler->quaternion :ax (/  pi  2))
;;                     )))
;;         (desig:a motion (type moving-tcp) (left-pose ?handle-pose))))


;;      )

;;    )
;;   )


(defun fetch-knife ()

  (let* ((?fetching-location (desig:a location
                                     (in (desig:an object
                                                   (type drawer)
                                                   (urdf-name sink-area-left-upper-drawer-main)
                                                   (owl-name "drawer_sinkblock_upper_open")
                                                   (part-of environment)))
                                     (side front)))
         (?object-designator
          (desig:an object
                    (type spoon)
                    ;; (type knife)
                    (location ?fetching-location)))

         (?pose (cl-transforms-stamped:make-pose-stamped
                 "map" 0.0
                 (cl-transforms:make-3d-vector 0.21 0.90 0)
                 (cl-transforms:make-quaternion 0 0 0.21 0.978)
                 ;; (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (0))
                 ;; (cl-transforms:make-identity-rotation)
                 ))
         )
      (exe:perform (desig:an action
                           (type accessing)
                           (arm left)
                           (location ?fetching-location)))

      (let ((?perceived-object-designator
             (exe:perform (desig:an action
                                    (type searching)
                                    (object ?object-designator)
                                    (location ?fetching-location)
                                                ))))
        (roslisp:ros-info (pp-plans transport)
                          "Found object of type ~a."
                          (desig:desig-prop-value ?perceived-object-designator :type))
        (let ((?fetched-object
                  (exe:perform (desig:an action
                                         (type fetching)
                                         (arms (right))
                                         ;; (grasp top)
                                         (object ?perceived-object-designator)
                                         (robot-location (a location (pose ?pose)))
                                         ))))
             (roslisp:ros-info (pp-plans transport) "Fetched the object.")


        (exe:perform (desig:an action
                           (type positioning-arm)
                           (right-configuration park)))
        (exe:perform (desig:an action
                             (type sealing)
                             (location ?fetching-location)))
        (exe:perform (desig:an action
                           (type positioning-arm)
                           (left-configuration park)))
        ;; If running on the real robot, execute below task tree in projection
        ;; N times first, then pick the best parameterization
        ;; and use that parameterization in the real world.
        ;; If running in projection, just execute the task tree below as normal.
        ;; (let (?fetch-pick-up-action ?deliver-place-action)
        ;;   (proj-reasoning:with-projected-task-tree
        ;;    (?fetch-robot-location ?fetch-pick-up-action
        ;;                           ?deliver-robot-location ?deliver-place-action)
        ;;    3
        ;;    #'proj-reasoning:pick-best-parameters-by-distance

        ;;    (let ((?fetched-object
        ;;           (exe:perform (desig:an action
        ;;                                  (type fetching)
        ;;                                  (arms (left))
        ;;                                  ;; (grasp top)
        ;;                                  (object ?perceived-object-designator)
        ;;                                  (pick-up-action ?fetch-pick-up-action)))))
        ;;      (roslisp:ros-info (pp-plans transport) "Fetched the object.")

        ;;      )))
        (return-from fetch-knife ?fetched-object)
        ;; (with-fields (answer)
        ;;              (value result)
        ;;              answer
        ;;              )
        )
        )
      )
  )


(defun fetch-topping ()

  (let* ((?fetching-location (desig:a location
                                      (on (desig:an object
                                                    (type counter-top)
                                                    (urdf-name sink-area-surface)
                                                    (owl-name "kitchen_sink_block_counter_top")
                                                    (part-of environment)))
                                      (side left)
                                      (side front)
                                      (range 0.5)))
         (?object-designator
          (desig:an object
                    (type cup)
                    (location ?fetching-location))
          )

         (?pose (cl-transforms-stamped:make-pose-stamped
                 "map" 0.0
                 ;; (cl-transforms:make-3d-vector 0.56112 1.0401 0)
                 (cl-transforms:make-3d-vector 0.6 0.5 0)
                 ;; (cl-transforms:make-3d-vector 0.56112 1.2401 0)
                 ;; (cl-transforms:make-quaternion 0 0 -0.076819 0.99705)
                 (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) 0.3)
                 ;; (cl-transforms:make-identity-rotation)
                 ))


         )

    (let ((?perceived-object-designator
           (exe:perform (desig:an action
                                  (type searching)
                                  (object ?object-designator)
                                  (location ?fetching-location)
                                  ))))
      (roslisp:ros-info (pp-plans transport)
                        "Found object of type ~a."
                        (desig:desig-prop-value ?perceived-object-designator :type))

      ;; base footprint 0.6441, 0.87709, 0       0, 0, 0.16296, 0.98663
      (let ((?fetched-object
             (exe:perform (desig:an action
                                    (type fetching)
                                    (arms (left))
                                    ;; (grasp top)
                                    (object ?perceived-object-designator)
                                    (robot-location (a location (pose ?pose)))
                                    ))))
        (roslisp:ros-info (pp-plans transport) "Fetched the object.")

        (exe:perform (desig:an action
                             (type positioning-arm)
                             (right-configuration park)))
        (return-from fetch-topping ?fetched-object)
        )
      )
    )
)



(defun cut-waffle ()
  (let*
      ((?waffle-location (desig:a location
                                    (on (desig:an object
                                                  (type counter-top)
                                                  (urdf-name kitchen-island-surface)
                                                  (owl-name "kitchen_island_counter_top")
                                                  (part-of kitchen)))
                                    ))
         (?object-designator
          (desig:an object
                    (type plate)
                    (location ?waffle-location)))

          (?pose (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -0.02 0.40 0)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 1))
                    ))

          (?robot-location-designator (desig:a location (pose ?pose)))

          (?first-cut-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.73 -0.2 1.05)
                    (cl-transforms:euler->quaternion :ax (/  pi  1.2))
                    ))

          (?first-cut-location-low (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.73 -0.2 1.00)
                    (cl-transforms:euler->quaternion :ax (/  pi  1.2))
                    ))

          (?second-cut-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.73 -0.05 1.05)
                                       (cl-transforms:euler->quaternion :ax (/  pi  1.2))
                    ))
          (?second-cut-location-low (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.73 -0.05 1.00)
                                       (cl-transforms:euler->quaternion :ax (/  pi  1.2))
                    ))

          (?third-cut-location (cl-tf:make-pose-stamped
                                cram-tf:*robot-base-frame* 0.0
                                (cl-transforms:make-3d-vector 0.75 -0.15 1.05)
                                (cl-transforms:euler->quaternion :ax (/  pi  1.2)  :az (/  pi  2))
                                ))
          (?third-cut-location-low (cl-tf:make-pose-stamped
                                    cram-tf:*robot-base-frame* 0.0
                                    (cl-transforms:make-3d-vector 0.75 -0.15 1.00)
                                    (cl-transforms:euler->quaternion :ax (/  pi  1.2)  :az (/  pi  2))
                                    ))

          (?fourth-cut-location (cl-tf:make-pose-stamped
                                 cram-tf:*robot-base-frame* 0.0
                                 (cl-transforms:make-3d-vector 0.6 -0.15 1.05)
                                 (cl-transforms:euler->quaternion :ax (/  pi  1.2)  :az (/  pi  2))
                                 ))
          (?fourth-cut-location-low (cl-tf:make-pose-stamped
                                     cram-tf:*robot-base-frame* 0.0
                                     (cl-transforms:make-3d-vector 0.6 -0.15 1.00)
                                     (cl-transforms:euler->quaternion :ax (/  pi  1.2)  :az (/  pi  2))
                                     ))

          )

    (exe:perform (desig:an action
                (type going)
                (target ?robot-location-designator)))

    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?first-cut-location)))
    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?first-cut-location-low)))
    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?second-cut-location-low)))
    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?second-cut-location)))

    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?third-cut-location)))
    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?third-cut-location-low)))
    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?fourth-cut-location-low)))
    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?fourth-cut-location)))

  )
)

(defun ask-preference ()

  (let* ((?pose (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector 0.41 1.01 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 1))
                    ))
          (?robot-location-designator (desig:a location (pose ?pose)))
         )
    (exe:perform (desig:an action
                (type going)
                (target ?robot-location-designator)))
   (let ((testanswer (aia:call-qna-action "Do you want vanilla toping?")))
     ;; (string= testanswer "Yes")
     (if (string= testanswer "Yes")
         (let ((?topping (fetch-topping)))
           (return-from ask-preference ?topping)
           )
       ;; (print "answer was yes")
       (let ((answer2 (aia:call-qna-action "Do you want blueberry toping?")))
         (if (string= answer2 "Yes")
             (let ((?topping (fetch-topping)))
               (return-from ask-preference ?topping)
               )
           (print "Ok, than no topping it is."))
         ))

     )
   )
  ;; (fetch-topping)
  )

(defun apply-topping ()
  (let*
      ((?waffle-location (desig:a location
                                    (on (desig:an object
                                                  (type counter-top)
                                                  (urdf-name kitchen-island-surface)
                                                  (owl-name "kitchen_island_counter_top")
                                                  (part-of kitchen)))
                                    ))
         (?object-designator
          (desig:an object
                    (type plate)
                    (location ?waffle-location)))

          (?pose (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector 0.05 0.48 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 1))
                    ))

          (?robot-location-designator (desig:a location (pose ?pose)))

          (?topping-approach-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.75 -0.2 1.05)
                    (cl-transforms:make-identity-rotation)
                    ;; (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 1 0 0) (/ pi 0))
                    ))
          (?topping-target-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.75 -0.2 1.05)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 1 0 0) (/ pi -1.5))
                    ))


          )

    (exe:perform (desig:an action
                (type going)
                (target ?robot-location-designator)))

    (exe:perform
       (desig:a motion (type moving-tcp) (left-pose ?topping-approach-location)))

    (exe:perform
       (desig:a motion (type moving-tcp) (left-pose ?topping-target-location)))

    (exe:perform
       (desig:a motion (type moving-tcp) (left-pose ?topping-approach-location))))
  )

(defun place-knife (?knife)
  (let* (
        (?pose (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -0.00 1.4 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 1))
                    ))

          (?robot-location-designator (desig:a location (pose ?pose)))

        (?drop-target-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.75 -0.1 1.00)
                   ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 1 0) (/ pi 2))
                    ))

        (?drop-target-location-low (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.75 -0.1 0.90)
                   ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 1 0) (/ pi 2))
                    ))


        )

    (exe:perform (desig:an action
                (type going)
                (target ?robot-location-designator)))
    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?drop-target-location)))

    (exe:perform
       (desig:a motion (type moving-tcp) (right-pose ?drop-target-location-low)))

    (exe:perform (desig:an action (type opening-gripper) (gripper right)))

    (cram-occasions-events:on-event
     (make-instance 'cpoe:object-detached-robot
                    :arm :right
                    :object-name (desig:desig-prop-value ?knife :name)))

    (exe:perform (desig:an action
                             (type positioning-arm)
                             (right-configuration park)
                             ))
    )

  )

(defun place-topping (?topping)
  (let* (

         (?pose (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -0.00 1.4 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 1))
                    ))

          (?robot-location-designator (desig:a location (pose ?pose)))

        (?drop-target-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.75 0.1 1.05)
                   (cl-transforms:make-identity-rotation)
                    ;; (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 1 0) (/ pi 2))
                    ))

        (?drop-target-location-low (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.75 0.1 0.95)
                    ;; (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 1 0) (/ pi 2))
                   (cl-transforms:make-identity-rotation)
                    ))
        )

    (exe:perform (desig:an action
                (type going)
                (target ?robot-location-designator)))
    (exe:perform
       (desig:a motion (type moving-tcp) (left-pose ?drop-target-location)))

    (exe:perform
       (desig:a motion (type moving-tcp) (left-pose ?drop-target-location-low)))

    (exe:perform (desig:an action (type opening-gripper) (gripper left)))

    (cram-occasions-events:on-event
     (make-instance 'cpoe:object-detached-robot
                    :arm :left
                    :object-name (desig:desig-prop-value ?topping :name)))

    (exe:perform (desig:an action
                             (type positioning-arm)
                             (left-configuration park)))
    )

  )

(defun deliver-waffle ()
  (let* ((?plate-location (desig:a location
                                   (side right)
                                   ;; (side front)
                                    (on (desig:an object
                                                  (type counter-top)
                                                  (urdf-name kitchen-island-surface)
                                                  (owl-name "kitchen_island_counter_top")
                                                  (part-of environment)))
                                    ))
         (?object-designator
          (desig:an object
                    (type plate)
                    ;; (type knife)
                    (location ?plate-location)))

         (?pose (cl-transforms-stamped:make-pose-stamped
                 "map" 0.0
                 (cl-transforms:make-3d-vector -0.144 0.054 0)
                 (cl-transforms:make-quaternion 0 0 0.94062 0.33946)
                 ;; (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (0))
                 ;; (cl-transforms:make-identity-rotation)
                 ))
        (?transfort-pose (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.55 0.4 1.25)
                   ;; (cl-transforms:make-identity-rotation)
                    ;; (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 1 0 0) (/ pi -2))
                    (cl-transforms:euler->quaternion :ax (/  pi  -2) :ay (/ pi 6))
                    ))
         )

    (let ((?perceived-object-designator
           (exe:perform (desig:an action
                                  (type searching)
                                  (object ?object-designator)
                                  (location ?plate-location)
                                  ))))
      (roslisp:ros-info (pp-plans transport)
                        "Found object of type ~a."
                        (desig:desig-prop-value ?perceived-object-designator :type))

      (let*((?fetched-object
             (exe:perform (desig:an action
                                    (type fetching)
                                    (arms (left))
                                    ;; (grasp top)
                                    (object ?perceived-object-designator)
                                    (robot-location (a location (pose ?pose)))
                                    )))

          (?deliver-pose1 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -1.30 -0.32 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 2))
                    ))

          (?deliver-pose2 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -2.30 -0.32 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 2))
                    ))

          (?deliver-pose3 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -4.30 -0.32 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 2))
                    ))
          (?deliver-pose4 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -4.80 0.32 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 4))
                    ))
          (?deliver-pose5 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -5.00 0.12 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 4))
                    ))
          (?deliver-location-designator1 (desig:a location (pose ?deliver-pose1)))
          (?deliver-location-designator2 (desig:a location (pose ?deliver-pose2)))
          (?deliver-location-designator3 (desig:a location (pose ?deliver-pose3)))
          (?deliver-location-designator4 (desig:a location (pose ?deliver-pose4)))
          (?deliver-location-designator5 (desig:a location (pose ?deliver-pose5)))

          (?waffel-target-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.65 0.1 0.85)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 1 0 0) (/ pi -2))
                    ))

          (?waffel-target-location-low (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.65 0.1 0.79)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 1 0 0) (/ pi -2))
                    ))

            )



        (roslisp:ros-info (pp-plans transport) "Fetched the object.")

        (exe:perform (desig:an action
                             (type positioning-arm)
                             (right-configuration park)
                             ))
        (exe:perform
         (desig:a motion (type moving-tcp) (left-pose ?transfort-pose)))

        (exe:perform (desig:an action
                (type going)
                (target ?deliver-location-designator1)))

        (exe:perform (desig:an action
                (type going)
                (target ?deliver-location-designator2)))
        (exe:perform (desig:an action
                (type going)
                (target ?deliver-location-designator3)))
        (exe:perform (desig:an action
                (type going)
                (target ?deliver-location-designator4)))

        (exe:perform
         (desig:a motion (type moving-tcp) (left-pose ?waffel-target-location)))

        (exe:perform
         (desig:a motion (type moving-tcp) (left-pose ?waffel-target-location-low)))

        (exe:perform (desig:an action (type opening-gripper) (gripper (left right))))

        (exe:perform (desig:an action
                (type going)
                (target ?deliver-location-designator5)))
        (park-robot)
        )

      )

        ;; (exe:perform
        ;;  (desig:an action
        ;;            (type transporting)
        ;;            (context table-setting)
        ;;            (object ?object-to-fetch)
        ;;            ;; (desig:when ?arm-to-use
        ;;            ;;   (arms (?arm-to-use)))
        ;;            ))


    )

  )

(defun transport-waffel()
      (let*(

          (?deliver-pose1 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -1.30 -0.32 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 2))
                    ))

          (?deliver-pose2 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -2.30 -0.32 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 2))
                    ))

          (?deliver-pose3 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -3.30 -0.32 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 2))
                    ))
          (?deliver-pose4 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -4.80 0.32 0)
                  ;; (cl-transforms:make-3d-vector -0.25 0.933 0)
                    ;; (cl-transforms:make-identity-rotation)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 4))
                    ))
          (?deliver-location-designator1 (desig:a location (pose ?deliver-pose1)))
          (?deliver-location-designator2 (desig:a location (pose ?deliver-pose2)))
          (?deliver-location-designator3 (desig:a location (pose ?deliver-pose3)))
          (?deliver-location-designator4 (desig:a location (pose ?deliver-pose4)))

          (?waffel-target-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.65 0.1 0.85)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 1 0 0) (/ pi 2))
                    ))

          (?waffel-target-location-low (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.65 0.1 0.79)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 1 0 0) (/ pi 2))
                    ))
            )


        (exe:perform (desig:an action
                (type going)
                (target ?deliver-location-designator1)))

        (exe:perform (desig:an action
                (type going)
                (target ?deliver-location-designator2)))
        (exe:perform (desig:an action
                (type going)
                (target ?deliver-location-designator3)))
        (exe:perform (desig:an action
                (type going)
                (target ?deliver-location-designator4)))

        (exe:perform
         (desig:a motion (type moving-tcp) (left-pose ?waffel-target-location)))

        (exe:perform
         (desig:a motion (type moving-tcp) (left-pose ?waffel-target-location-low)))
        )
  )

(defun init-avatar ()
  ;; (aia:start-ros-node "cram_client")
  ;; (aia:init-ros-avatar "avatar")
  )

(defun ropha-demo-waffle ()
  (pr2-pms:with-real-robot
  ;; (urdf-proj:with-simulated-robot
    (demo::initialize)
    (init-avatar)
    ;; (when cram-projection:*projection-environment*
    ;;     (spawn-objects-on-sink-counter :random nil))
    (park-robot)


    ;; (deliver-waffle)
    ;; (let (
    ;;       (?topping (ask-preference)))
    ;;   (place-topping ?topping)
    ;;   )
    ;; (let (
    ;;       (?knife (fetch-knife)))
    ;;   (place-knife ?knife)
    ;;   )
    (let ((?topping (ask-preference))
          (?knife (fetch-knife)))
      (apply-topping)
      (cut-waffle)
      (place-knife ?knife)
      (place-topping ?topping)
      (deliver-waffle)
      )


    ;; (ask-preference)
    ;; (fetch-topping)
    ;; (place-topping)
    ;; (apply-topping)
    ;; (cut-waffle)
    ;; (deliver-waffle)
    ;; (transport-waffel)



    ;; (print "Ask for preference")
    ;; (ask-preference)

   )
)

;; (defun ropha-demo ()
;;   (pr2-pms:with-real-robot
;;   (initialize)
;;   (park-robot)

;;    (let* (
;;           (?perceived-object-designator
;;            (cram-executive:perform
;;             (desig:an action
;;                       (type detecting)
;;                       (object (desig:an object (type spoon)))))
;;            )

;;           (?pose (cl-transforms-stamped:make-pose-stamped
;;                   "map" 0.0
;;                   (cl-transforms:make-3d-vector -4.55 0.233 0)
;;                   (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 4))))

;;           (?robot-location-designator (desig:a location (pose ?pose))
;;                                       )
;;           (?bowl-pose (cl-transforms-stamped:make-pose-stamped
;;                   "map" 0.0
;;                   (cl-transforms:make-3d-vector -4.1 0.7 0.88)
;;                   (cl-transforms:make-identity-rotation)))

;;           (?bowl-location-designator (desig:a location (pose ?bowl-pose))
;;                                       )

;;           (?mouth-pose (cl-transforms-stamped:make-pose-stamped
;;                   "map" 0.0
;;                   (cl-transforms:make-3d-vector -4.36 1.06 1.11)
;;                   (cl-transforms:make-identity-rotation)))

;;           (?mouth-location-designator (desig:a location (pose ?mouth-pose))
;;                                       )
;;           )
;;      (cram-executive:perform
;;       (desig:an action
;;                 (type fetching)
;;                 (arm left)
;;                 (object ?perceived-object-designator)
;;                 ))

;;      (cpl:with-failure-handling
;;       ((cpl:plan-failure (e)
;;                          (declare (ignore e))
;;                          (return)))

;;        (cram-executive:perform
;;         (desig:an action
;;                 (type going)
;;                 (target ?robot-location-designator))))

;;      (cram-executive:perform
;;       (desig:an action
;;                 (type spooning)
;;                 (location ?bowl-location-designator)
;;                 (target ?mouth-location-designator)
;;                 ))
;;      )

;;   (finalize)
;;    )
;; )

;; (cpl:def-cram-function demo-random (&optional
;;                                     (random t)
;;                                     (list-of-objects '(:bowl :spoon :cup :milk :breakfast-cereal)))

;;   (initialize)
;;   (when cram-projection:*projection-environment*
;;     (if random
;;         (spawn-objects-on-sink-counter-randomly)
;;         (spawn-objects-on-sink-counter)))

;;   (park-robot)

;;   (let ((object-fetching-locations
;;           `((:breakfast-cereal . ,(desig:a location
;;                                            (on (desig:an object
;;                                                          (type counter-top)
;;                                                          (urdf-name sink-area-surface)
;;                                                          (owl-name "kitchen_sink_block_counter_top")
;;                                                          (part-of kitchen)))
;;                                            (side left)
;;                                            (side front)
;;                                            (range 0.5)))
;;             (:cup . ,(desig:a location
;;                               (side left)
;;                               (on (desig:an object
;;                                             (type counter-top)
;;                                             (urdf-name sink-area-surface)
;;                                             (owl-name "kitchen_sink_block_counter_top")
;;                                             (part-of kitchen)))))
;;             (:bowl . ,(desig:a location
;;                                (on (desig:an object
;;                                              (type counter-top)
;;                                              (urdf-name sink-area-surface)
;;                                              (owl-name "kitchen_sink_block_counter_top")
;;                                              (part-of kitchen)))
;;                                (side left)
;;                                (side front)
;;                                (range-invert 0.5)))
;;             (:spoon . ,(desig:a location
;;                                 (in (desig:an object
;;                                               (type drawer)
;;                                               (urdf-name sink-area-left-upper-drawer-main)
;;                                               (owl-name "drawer_sinkblock_upper_open")
;;                                               (part-of kitchen)))
;;                                 (side front)))
;;             (:milk . ,(desig:a location
;;                                (side left)
;;                                (side front)
;;                                (range 0.5)
;;                                (on;; in
;;                                    (desig:an object
;;                                              (type counter-top)
;;                                              (urdf-name sink-area-surface ;; iai-fridge-main
;;                                                         )
;;                                              (owl-name "kitchen_sink_block_counter_top"
;;                                                        ;; "drawer_fridge_upper_interior"
;;                                                        )
;;                                              (part-of kitchen)))))))
;;         (object-placing-locations
;;           (let ((?pose
;;                   (cl-transforms-stamped:make-pose-stamped
;;                    "map"
;;                    0.0
;;                    (cl-transforms:make-3d-vector -0.78 0.8 0.95)
;;                    (cl-transforms:make-quaternion 0 0 0.6 0.4))))
;;             `((:breakfast-cereal . ,(desig:a location
;;                                              (pose ?pose)
;;                                              ;; (left-of (an object (type bowl)))
;;                                              ;; (far-from (an object (type bowl)))
;;                                              ;; (for (an object (type breakfast-cereal)))
;;                                              ;; (on (desig:an object
;;                                              ;;               (type counter-top)
;;                                              ;;               (urdf-name kitchen-island-surface)
;;                                              ;;               (owl-name "kitchen_island_counter_top")
;;                                              ;;               (part-of kitchen)))
;;                                              ;; (side back)
;;                                              ))
;;               (:cup . ,(desig:a location
;;                                 (right-of (an object (type bowl)))
;;                                 ;; (behind (an object (type bowl)))
;;                                 (near (an object (type bowl)))
;;                                 (for (an object (type cup)))))
;;               (:bowl . ,(desig:a location
;;                                  (on (desig:an object
;;                                                (type counter-top)
;;                                                (urdf-name kitchen-island-surface)
;;                                                (owl-name "kitchen_island_counter_top")
;;                                                (part-of kitchen)))
;;                                  (context table-setting)
;;                                  (for (an object (type bowl)))
;;                                  (object-count 3)
;;                                  (side back)
;;                                  (side right)
;;                                  (range-invert 0.5)))
;;               (:spoon . ,(desig:a location
;;                                   (right-of (an object (type bowl)))
;;                                   (near (an object (type bowl)))
;;                                   (for (an object (type spoon)))))
;;               (:milk . ,(desig:a location
;;                                  (left-of (an object (type bowl)))
;;                                  (far-from (an object (type bowl)))
;;                                  (for (an object (type milk)))))))))

;;     (an object
;;         (obj-part "drawer_sinkblock_upper_handle"))



;;     (dolist (?object-type list-of-objects)
;;       (let* ((?fetching-location
;;                (cdr (assoc ?object-type object-fetching-locations)))
;;              (?delivering-location
;;                (cdr (assoc ?object-type object-placing-locations)))
;;              (?arm-to-use
;;                (cdr (assoc ?object-type *object-grasping-arms*)))
;;              (?cad-model
;;                (cdr (assoc ?object-type *object-cad-models*)))
;;              (?color
;;                (cdr (assoc ?object-type *object-colors*)))
;;              (?object-to-fetch
;;                (desig:an object
;;                          (type ?object-type)
;;                          (location ?fetching-location)
;;                          (desig:when ?cad-model
;;                            (cad-model ?cad-model))
;;                          (desig:when ?color
;;                            (color ?color)))))


;;         (when (eq ?object-type :bowl)
;;           (cpl:with-failure-handling
;;               ((common-fail:high-level-failure (e)
;;                  (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping the search" e)
;;                  (return)))
;;             (let ((?loc (cdr (assoc :breakfast-cereal object-fetching-locations))))
;;               (exe:perform
;;                (desig:an action
;;                          (type searching)
;;                          (object (desig:an object (type breakfast-cereal)))
;;                          (location ?loc))))))

;;         (cpl:with-failure-handling
;;             ((common-fail:high-level-failure (e)
;;                (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
;;                (return)))
;;           (if (eq ?object-type :bowl)
;;               (exe:perform
;;                (desig:an action
;;                          (type transporting)
;;                          (object ?object-to-fetch)
;;                          ;; (arm right)
;;                          (location ?fetching-location)
;;                          (target ?delivering-location)))
;;               (if (eq ?object-type :breakfast-cereal)
;;                   (exe:perform
;;                    (desig:an action
;;                              (type transporting)
;;                              (object ?object-to-fetch)
;;                              ;; (arm right)
;;                              (location ?fetching-location)
;;                              (target ?delivering-location)))
;;                   (exe:perform
;;                    (desig:an action
;;                              (type transporting)
;;                              (object ?object-to-fetch)
;;                              (desig:when ?arm-to-use
;;                                (arm ?arm-to-use))
;;                              (location ?fetching-location)
;;                              (target ?delivering-location))))))

;;         ;; (setf pr2-proj-reasoning::*projection-reasoning-enabled* nil)
;;         )))

;;   ;; (setf pr2-proj-reasoning::*projection-reasoning-enabled* nil)

;;   (park-robot)

;;   (finalize)

;;       cpl:*current-path*
;;       )
