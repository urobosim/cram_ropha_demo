(in-package :ropha-demo)

(def-fact-group robot-action-designators (action-grounding)

  ;; (<- (desig:action-grounding ?desig (ask-preference ?desig))
  ;;   (desig-prop ?desig (:type  :ask-preference))
  ;;   )

  (<- (desig:action-grounding ?desig (cut-waffle ?desig))
    (desig-prop ?desig (:type  :cut-waffle))
    )


  (<- (desig:action-grounding ?desig (deliver-waffle ?desig))
    (desig-prop ?desig (:type  :deliver-waffle))
    )

  (<- (desig:action-grounding ?desig (place-knife ?desig))
      (desig-prop ?desig (:type  :place-knife)
      (desig-prop ?desig (:object ?knife)))
    )

  (<- (desig:action-grounding ?desig (fetch-knife ?desig))
    (desig-prop ?desig (:type  :fetch-knife ))
    )

  (<- (desig:action-grounding ?desig (fetch-topping ?desig))
      (desig-prop ?desig (:type  :fetch-topping )
                  ;; (desig-prop ?desig (:string answer))
                  )
    )

  (<- (desig:action-grounding ?desig (apply-topping ?desig))
    (desig-prop ?desig (:type  :apply-topping))
    )

  (<- (desig:action-grounding ?desig (place-topping ?desig))
      (desig-prop ?desig (:type  :place-topping)
      (desig-prop ?desig (:object ?topping)))
    )
  )
