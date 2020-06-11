(in-package :ropha-demo)

(defun apply-topping (&key &allow-other-keys)
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

(defun cut-waffle (&key &allow-other-keys)
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
                   (cl-transforms:make-3d-vector 0.73 -0.2 0.99)
                    (cl-transforms:euler->quaternion :ax (/  pi  1.2))
                    ))

          (?second-cut-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.73 -0.05 1.05)
                                       (cl-transforms:euler->quaternion :ax (/  pi  1.2))
                    ))
          (?second-cut-location-low (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.73 -0.05 1.99)
                                       (cl-transforms:euler->quaternion :ax (/  pi  1.2))
                    ))

          (?third-cut-location (cl-tf:make-pose-stamped
                                cram-tf:*robot-base-frame* 0.0
                                (cl-transforms:make-3d-vector 0.75 -0.15 1.05)
                                (cl-transforms:euler->quaternion :ax (/  pi  1.2)  :az (/  pi  2))
                                ))
          (?third-cut-location-low (cl-tf:make-pose-stamped
                                    cram-tf:*robot-base-frame* 0.0
                                    (cl-transforms:make-3d-vector 0.75 -0.15 0.99)
                                    (cl-transforms:euler->quaternion :ax (/  pi  1.2)  :az (/  pi  2))
                                    ))

          (?fourth-cut-location (cl-tf:make-pose-stamped
                                 cram-tf:*robot-base-frame* 0.0
                                 (cl-transforms:make-3d-vector 0.6 -0.15 1.05)
                                 (cl-transforms:euler->quaternion :ax (/  pi  1.2)  :az (/  pi  2))
                                 ))
          (?fourth-cut-location-low (cl-tf:make-pose-stamped
                                     cram-tf:*robot-base-frame* 0.0
                                     (cl-transforms:make-3d-vector 0.6 -0.15 0.99)
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
;; ((:string answer))
(defun fetch-topping (&key  &allow-other-keys)

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
