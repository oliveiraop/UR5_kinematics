# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import time

from zmqRemoteApi import RemoteAPIClient
import math

print('Program started')


def movCallback(config,vel,accel,handles):
    for i in range(len(handles)):
        if sim.getJointMode(handles[i])[0]==sim.jointmode_force and sim.isDynamicallyEnabled(handles[i]):
            sim.setJointTargetPosition(handles[i],config[i])
        else:
            sim.setJointPosition(handles[i],config[i])

client = RemoteAPIClient()
sim = client.getObject('sim')
jointHandles = [ 
    sim.getObject('/UR5/UR5_joint1'),
    sim.getObject('/UR5/UR5_joint2'),
    sim.getObject('/UR5/UR5_joint3'),
    sim.getObject('/UR5/UR5_joint4'),
    sim.getObject('/UR5/UR5_joint5'),
    sim.getObject('/UR5/UR5_joint6'),
    ]
# When simulation is not running, ZMQ message handling could be a bit
# slow, since the idle loop runs at 8 Hz by default. So let's make
# sure that the idle loop runs at full speed for this program:
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)

mVel = 100 * math.pi / 180
mAccel = 150 * math.pi / 180
mJerk = 100 * math.pi / 180
maxVel=[mVel, mVel, mVel, mVel, mVel, mVel]
maxAccel=[mAccel, mAccel, mAccel, mAccel, mAccel, mAccel]
maxJerk=[mJerk, mJerk, mJerk, mJerk, mJerk, mJerk]

sim.startSimulation()


currentConf = [ 0, 0, 0, 0, 0, 0 ]

targetConfig = [
    (300 * math.pi / 180), (14.78 * math.pi / 180) + math.pi/2,  63.25 * math.pi / 180 , -78.04 * math.pi / 180, (180 * math.pi / 180) + math.pi/2, 59 * math.pi / 180
]

sim.moveToConfig(-1,currentConf,None,None,maxVel,maxAccel,maxJerk,targetConfig,None,movCallback,jointHandles)

time.sleep(5)


shape = sim.getObject('/Cup')
tool_matrix = sim.getObjectMatrix(shape, sim.handle_world)
print(f"Matrix copo = {tool_matrix}")

tool = sim.getObject('/UR5/UR5_connection')
tool_position = sim.getObjectPosition(tool, sim.handle_world)
print(f"posicao_garra_sim = {tool_position}")
tool_orientation = sim.getObjectOrientation(tool, sim.handle_world)
print(f"orientation = {tool_orientation}")
tool_matrix = sim.getObjectMatrix(tool, sim.handle_world)
print(f"Matrix = {tool_matrix}")

"""currentConf = targetConfig
targetConfig = [
    -90 * math.pi / 180, -45 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180
]

sim.moveToConfig(-1,currentConf,None,None,maxVel,maxAccel,maxJerk,targetConfig,None,movCallback,jointHandles)
 """
sim.stopSimulation()

# Run a simulation in asynchronous mode:
""" 
while (t := sim.getSimulationTime()) < 10:
    s = f'Simulation time: {t:.2f} [s] (simulation running asynchronously '\
        'to client, i.e. non-stepped)'
    print(s)
    sim.setJointPosition(joint1, 90 * math.pi / 180)
    sim.addLog(sim.verbosity_scriptinfos, s)
sim.stopSimulation()
# If you need to make sure we really stopped:
while sim.getSimulationState() != sim.simulation_stopped:
    time.sleep(0.1) """
"""
# Run a simulation in stepping mode:
client.setStepping(True)
sim.startSimulation()
while (t := sim.getSimulationTime()) < 3:
    s = f'Simulation time: {t:.2f} [s] (simulation running synchronously '\
        'to client, i.e. stepped)'
    print(s)
    sim.addLog(sim.verbosity_scriptinfos, s)
    client.step()  # triggers next simulation step
sim.stopSimulation()
"""
# Remove the dummies created earlier:

# Restore the original idle loop frequency:
sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

print('Program ended')
