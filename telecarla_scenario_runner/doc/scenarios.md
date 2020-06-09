# Scenarios

## StationaryObjectCrossing

* total timeout 60 seconds
* scenario sequence:
  * obstacle is spawned on street
  * wait 15 seconds (ego can do anything)
  * obstacle is removed
  * _then_, ego vehicle has to drive 40 meters for scenario to end

## DynamicObjectCrossing

* total timeout 60 seconds
* scenario sequence:
  * walker triggered when ETA is below certain threshold (triggered almost immediately due to low distance)
  * walker accelerates and crosses street
  * walker reaches other side of road
  * _then_, ego vehicle has to drive 45 meters for scenario to end

## ManeuverOppositeDirection

* total timeout 120 seconds
* ego vehicle has to drive 220 meters for scenario to end

## SignalizedJunctionLeftTurn

* total timeout 80 seconds
* ego vehicle has to drive 110 meters for scenario to end

## SignalizedJunctionRightTurn

* total timeout 80 seconds
* scenario sequence:
  * other vehicle is spawned and crosses in front of ego vehicle
  * other vehicle reaches destination and comes to a stop
  * _then_, ego vehicle has to drive 40 meters for scenario to end

## FollowLeadingVehicle

* total timeout 120 seconds
* scenario sequence:
  * leading vehicle drives to next intersection and stops
  * ego vehicle has to be within 20 meters of leader and has to make at least one complete stop for scenario to end

## FollowLeadingVehicleWithObstacle

* total timeout 120 seconds
* scenario sequence:
  * leading vehicle drives to next intersection and stops
  * ego vehicle has to be within 20 meters of leader and has to make at least one complete stop for scenario to end
