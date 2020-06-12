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

(defun place-knife (&key ((:object ?knife)) &allow-other-keys)
;; (defun place-knife (?knife)
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

(defun place-topping (&key ((:object ?topping)) &allow-other-keys)
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

(defun deliver-waffle (&key &allow-other-keys)
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
        ;; (?transfort-pose (cl-tf:make-pose-stamped
        ;;            cram-tf:*robot-base-frame* 0.0
        ;;            (cl-transforms:make-3d-vector 0.55 0.4 1.25)
        ;;            ;; (cl-transforms:make-identity-rotation)
        ;;             ;; (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 1 0 0) (/ pi -2))
        ;;             (cl-transforms:euler->quaternion :ax (/  pi  -2) :ay (/ pi 6))
        ;;             ))
         )

    ;; (let ()
    ;;   (roslisp:ros-info (pp-plans transport)
    ;;                     "Found object of type ~a."
    ;;                     (desig:desig-prop-value ?perceived-object-designator :type))

      (let*(
            (?perceived-object-designator
             (exe:perform (desig:an action
                                  (type searching)
                                  (object ?object-designator)
                                  (location ?plate-location)
                                  )))
            (?fetched-object
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
                  ;; (cl-transforms:make-3d-vector -4.80 0.32 0)
                  (cl-transforms:make-3d-vector -4.4 0.08 0)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 3))
                    ))
          (?deliver-pose5 (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms:make-3d-vector -4.80 -0.32 0)
                    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi 4))
                    ))
          (?deliver-location-designator1 (desig:a location (pose ?deliver-pose1)))
          (?deliver-location-designator2 (desig:a location (pose ?deliver-pose2)))
          (?deliver-location-designator3 (desig:a location (pose ?deliver-pose3)))
          (?deliver-location-designator4 (desig:a location (pose ?deliver-pose4)))
          (?deliver-location-designator5 (desig:a location (pose ?deliver-pose5)))

          (?waffel-target-location (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.68 0.37 0.85)
                    (cl-transforms:euler->quaternion :ax (/  pi  -2) :ay (/ pi 6))
                    ))

          (?waffel-target-location-low (cl-tf:make-pose-stamped
                   cram-tf:*robot-base-frame* 0.0
                   (cl-transforms:make-3d-vector 0.68 0.37 0.79)
                    (cl-transforms:euler->quaternion :ax (/  pi  -2) :ay (/ pi 6))
                    ))

            )



        (roslisp:ros-info (pp-plans transport) "Fetched the object.")

        (exe:perform (desig:an action
                             (type positioning-arm)
                             (right-configuration park)
                             ))
        ;; (exe:perform
        ;;  (desig:a motion (type moving-tcp) (left-pose ?transfort-pose)))

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


        (aia:send-sync-state 2)
        )

      ;; )

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

(defun transport-waffel(&key &allow-other-keys)
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

(defun fetch-knife (&key &allow-other-keys)

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
             ;; (return-from fetch-knife ?fetched-object)

        (desig:current-desig ?fetched-object))
        )

      )
  )
