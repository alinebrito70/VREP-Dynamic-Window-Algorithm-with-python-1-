import sys

COPPELIASIM_ZMQ_PATH = r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src"

if COPPELIASIM_ZMQ_PATH not in sys.path:
    sys.path.append(COPPELIASIM_ZMQ_PATH)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient(host='127.0.0.1', port=23000)
sim = client.getObject('sim')

print("Conectou no CoppeliaSim!")
print("Tempo de simulação:", sim.getSimulationTime())