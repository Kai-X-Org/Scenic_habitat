# Traffic Scenario 10: Crossing negotiation at an unsignalized intersection
# Definition: Ego-vehicle needs to negotiate with other vehicles to cross an unsignalized intersection. In this situation it is assumed that the first to enter the intersection has priority. 
# actor arrives at 4 way intersection before ego, has right of way. crosses intersection before ego.

from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import setMapPath
setMapPath(__file__, 'maps/borregasave.xodr')
from scenic.simulators.lgsvl.model import *
from scenic.simulators.lgsvl.behaviors import *
import matplotlib.pyplot as plt

simulator LGSVLSimulator('BorregasAve')
param time_step = 1.0/10

# CONSTANTS
fourLane = filter(lambda i: i.is4Way, network.intersections)
intersection = Uniform(*fourLane)

straight_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, intersection.maneuvers)
egoManeuver = Uniform(*straight_maneuvers)

randomType = Uniform(*[ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN])
conflicting_maneuvers = filter(lambda  m: m.type == ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers) # hard coded. should be m.type == randomType
actorTurn = Uniform(*conflicting_maneuvers)


egoAtStop = False


# GEOMETRY
lane1 = egoManeuver.startLane
lane2 = actorTurn.startLane

lines1 = [egoManeuver.startLane.centerline, egoManeuver.connectingLane.centerline, egoManeuver.endLane.centerline]
lines2 = [actorTurn.startLane.centerline, actorTurn.connectingLane.centerline, actorTurn.endLane.centerline]

egoTrajectory = [egoManeuver.connectingLane.centerline, egoManeuver.endLane.centerline]
actorTrajectory = [actorTurn.connectingLane.centerline, actorTurn.endLane.centerline]

pos1 = (OrientedPoint at lane1.centerline[-1]) offset by (-2, 2) @ 0 
pos2 = (OrientedPoint at lane2.centerline[-1]) offset by (-2, 2) @ 0

egoDist = (8,12)
actorDist = (5, 7)


# BEHAVIORS

behavior EgoBehavior(target_speed=20, trajectory = None):
	smallDistance = (5, 6)
	try: 
		FollowTrajectoryBehavior(target_speed = 15, trajectory = lines1)
		
	interrupt when ((distance from ego to intersection) <= smallDistance):
		take SetBrakeAction(1), SetThrottleAction(0.0)
		if(ego.speed <= 0.7):
			while (True):
				egoChicken(target_speed = 15, trajectory = egoTrajectory)

behavior egoChicken(target_speed = 15, trajectory = None):
	brakeIntensity = (0.9, 1.0)
	assert trajectory is not None
	try:
		FollowTrajectoryBehavior(target_speed = target_speed, trajectory = trajectory)
	interrupt when ((distance from ego to actorCar) <= 8):
		take SetBrakeAction(brakeIntensity)

behavior actorCarBehavior(egoAtStop):
	print("actor turn: ", actorTurn.type)
	randomBehavior = Uniform(*PossibleBehaviors)
	print("random behavior will be: ", randomBehavior)
	speedup = (0,1)
	smallDistance = (2, 3)
	try: 
		FollowTrajectoryBehavior(target_speed = 15, trajectory = lines2)
		print("moving")
	interrupt when ((distance from actorCar to intersection) <= smallDistance):
		while (egoAtStop):
			print("starting random behavior: ", randomBehavior)
			randomBehavior()
		print("actor stopping")
		take SetBrakeAction(1), SetThrottleAction(0.0)
		if (ego.speed <= 0.7): # if ego is stopped, actor will go
			egoAtStop = True
			print("ego at stop point")
		


behavior chickenBehavior(target_speed=20, trajectory = actorTrajectory):
	assert trajectory is not None
	brakeIntensity = (0.6, 0.8)

	network.show(plt)
	actorTurn.connectingLane.centerline.show(plt, 'b')
	plt.show()
	try: 
		FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)
	
	interrupt when ((ego.speed < 0.2) or ((distance from actorCar to ego) <= 7)): # actor starts when ego starts, stops when ego stops.
		print("interrupted")
		take SetReverseAction(False), SetThrottleAction(0.0), SetBrakeAction(brakeIntensity)


behavior neverMoveBehavior():
	while True:
		take SetThrottleAction(0.0), SetBrakeAction(1)

behavior conflictingStopBehavior(target_speed=20, trajectory = actorTrajectory): 	# actor begins doing its turn, but then stops in the intersection
	assert trajectory is not None
	take SetReverseAction(False)
	turn = actorTurn
	randomDist = (3, 5)
	brakeIntensity = (0.6, 0.8)

	try: 
		FollowTrajectoryBehavior(target_speed=15, trajectory=trajectory)
	
	interrupt when ((distance from ego to actorCar) <= randomDist):
		print("interrupted")
		take SetThrottleAction(0.0), SetBrakeAction(brakeIntensity)

behavior turnBehavior():
	take SetReverseAction(False)
	turn = actorTurn
	brakeIntensity = (0.6, 0.8)
	turnSpeed = (10,15)

	try: 
		FollowTrajectoryBehavior(target_speed=turnSpeed, trajectory=actorTrajectory)

	interrupt when((distance from actorCar to actorTrajectory[1][-1]) <= 1): # just needed to interrupt.
		print("done")
		abort

PossibleBehaviors = [neverMoveBehavior, turnBehavior, conflictingStopBehavior, chickenBehavior]
#PossibleBehaviors = [chickenBehavior]

# PLACEMENT
ego = EgoCar following roadDirection from pos1 by -egoDist,
	with behavior EgoBehavior(target_speed=15, trajectory=lines1)

actorCar = EgoCar following roadDirection from pos2 by -actorDist,
	with behavior actorCarBehavior(egoAtStop)

terminate when (ego in egoManeuver.endLane)
terminate when (actorCar in actorTurn.endLane)
