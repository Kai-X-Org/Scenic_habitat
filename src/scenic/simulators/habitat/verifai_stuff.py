from verifai.samplers.scenic_sampler import ScenicSampler
from verifai.falsifier import generic_falsifier
from dotmap import DotMap
from verifai.scenic_server import ScenicServer
from verifai.monitor import specification_monitor
# class scenic_spec(specification_monitor):
#     def __init__(self):
#         def specification(traj):
#             print(traj.result)
#             print(type(traj.result.trajectory))
#             print("TRAJ: " + str(traj.result.trajectory))
#             return True
#         super().__init__(specification)
path_to_scenic_script = './examples/unity/ObjectSpawnTest.scenic' # TODO replace with sample scenario to run
sampler = ScenicSampler.fromScenario(path_to_scenic_script)
MAX_ITERS = 1
PORT = 8888 # TODO not the same port that Scenic runs. Can leave as whatever
MAXREQS = 5
BUFSIZE = 4096
falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS # how many iter to run before spitting out results
falsifier_params.save_error_table = True # saving things
falsifier_params.save_good_samples = True # saving things
server_options = DotMap(bufsize=BUFSIZE, maxreqs=MAXREQS, verbosity=1)
falsifier = generic_falsifier(sampler=sampler,
                              falsifier_params=falsifier_params,
                              server_class=ScenicServer, # TODO required, need ScenicServer, need dynamic Scenic
                              server_options=server_options)
falsifier.run_falsifier()
print("Scenic Samples")
"""
below prints out what the falsifier is getting
"""
'''
if Scenic has an error, will not get traceback from Scenic

for i in falsifier.samples.keys(): # only look at what happens at the first timestep
    print("Sample: ", i)
    print(falsifier.samples[i])
    print("Headset Position: " + str(falsifier.samples[i].objects.object0.gameObject.position))
    print("LeftHand Position: " + str(falsifier.samples[i].objects.object0.gameObject.leftHandPosition))
    print("RightHand Position: " + str(falsifier.samples[i].objects.object0.gameObject.rightHandPosition))
'''
