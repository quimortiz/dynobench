import math
import numpy as np
import  rowan as rn
import yaml
import time
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import argparse

class Quad():
    def __init__(self, name, l):
        self.l = l # length of cable
        self.name = name    
        self.state = 0 # [quad_pos, quat, vel, w]
        self.state_ap = []
    
    def _updateState(self, state):
        self.state = state
        self.state_ap.append(state)

        
class Payload():
    def __init__(self, shape):
        self.shape = shape
        self.state = 0
        self.state_ap = []
    def _updateState(self, state):
        self.state = state
        self.state_ap.append(state)


class QuadPayloadRobot():
    def __init__(self, quadNum=1, pType="quad3dpayload"):
        ## ASSUMPTION: CABLE LENTH IS ALWAYS 0.5 
        self.quadNum = quadNum
        self.pType = pType
        l = (0.5*np.ones(quadNum,)).tolist()
        self._addQuad(l)
        self._addPayload()
    
    def _addQuad(self, l):
        self.quads = dict()
        for i in range(self.quadNum):
            self.quads[str(i)] = Quad(str(i), l[i])

    def _addPayload(self):
        self.payload = Payload(self.pType)

    def updateFullState(self, state):
        if self.payload.shape == "quad3dpayload":
            self.state = state.copy()
            _state = state.copy() # this is useful for the multiple uavs 
        elif self.payload.shape == "point":
            # THIS FOR MULTIPLE UAVs FOR POINTMASS PAYLOAD
            # NOT IMPLEMENTED
            print('NOT IMPLEMENTED, please use payload type as point')
            exit()
        elif self.payload.shape == "rigid":
            # THIS IS FOR MULTIPLE UAVS WITH RIGID PAYLOAD
            # NOT IMPLEMENTED
            print('NOT IMPLEMENTED, please use payload type as point')
            exit()
        else:
            print("Payload type doesn't exist")
            exit()

        self.payload._updateState(state)
        for name,quad in self.quads.items():
            self.quads[name] = self._updateRobotState(quad, _state)
        
    def _updateRobotState(self, quad, state):
        # position, quaternion, velocity, angular velocity
        p_load = np.array(state[0:3])
        qc     = np.array(state[3:6])
        v_load = np.array(state[6:9])
        wc     = np.array(state[9:12])
        qc_dot = np.cross(wc, qc)
        quat   = state[12:16]
        quat   = np.array([quat[3], quat[0], quat[1], quat[2]])
        w      = np.array(state[16:19])
        p_quad = p_load - quad.l*qc
        v_quad = v_load - quad.l*qc_dot
        quad_state = [*p_quad, *quat, *v_quad, *w]
        quad._updateState(quad_state)
        return quad

class Visualizer():
    def __init__(self, QuadPayloadRobot, obstacles):
        self.vis = meshcat.Visualizer()
        self.vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0.5, 0, 0]).dot(
            tf.euler_matrix(np.radians(-40), np.radians(0), np.radians(-100))))
        self.vis["/Cameras/default/rotated/<object>"].set_transform(
            tf.translation_matrix([-2, 0, 2.5]))
  
        self.QuadPayloadRobot = QuadPayloadRobot
        self._addQuadsPayload()
        self._setObstacles(obstacles)

    def _addQuadsPayload(self):
        self.quads = self.QuadPayloadRobot.quads
        self.payload = self.QuadPayloadRobot.payload
        if self.payload.shape == "quad3dpayload":    
            self.vis[self.payload.shape].set_object(g.Mesh(g.Sphere(0.02), 
                                    g.MeshLambertMaterial(color=0xff11dd)))
        elif self.payload.shape == "point":
            # THIS FOR MULTIPLE UAVs FOR POINTMASS PAYLOAD
            # different state order
            # NOT IMPLEMENTED
            print('NOT IMPLEMENTED, please use payload type as point')
            exit()
        elif self.payload.shape == "rigid":
            # THIS IS FOR MULTIPLE UAVS WITH RIGID PAYLOAD
            # different state order
            # NOT IMPLEMENTED
            print('NOT IMPLEMENTED, please use payload type as point')
            exit()
        else:
            print("Payload type doesn't exist")
            exit()

        for name in self.quads.keys():
            self.vis[name].set_object(g.StlMeshGeometry.from_file('cf2_assembly.stl'))
    
    def _setObstacles(self, obstacles):
        for idx, obstacle in enumerate(obstacles): 
            obsMat = g.MeshLambertMaterial(opacity=0.5, color=0x008000)
            center = obstacle["center"]
            shape = obstacle["type"]
            if (shape =="sphere"):
                radius = obstacle["radius"]
                self.vis["obstacle"+str(idx)].set_object(g.Mesh(g.Sphere(radius),material=obsMat))
                self.vis["obstacle"+str(idx)].set_transform(tf.translation_matrix(center))
            elif shape == "box":
                size = obstacle["size"]
                self.vis["obstacle"+str(idx)].set_object(g.Mesh(g.Box(size), material=obsMat))
                self.vis["obstacle"+str(idx)].set_transform(tf.translation_matrix(center))
    
    def updateVis(self, state):
        self.QuadPayloadRobot.updateFullState(state)
        payloadSt = self.payload.state
        # color of the payload trajectory
        point_color = np.array([1.0, 1.0, 1.0])
        full_state = np.array(self.payload.state_ap, dtype=np.float64)[:,0:3].T
        self.vis['points'].set_object(g.Points(
            g.PointsGeometry(full_state, color=point_color),
            g.PointsMaterial(size=0.01)
        ))
        if self.payload.shape == "quad3dpayload" or self.payload.shape == "point":
            self.vis[self.payload.shape].set_transform(
                    tf.translation_matrix(payloadSt).dot(
                        tf.quaternion_matrix([1,0,0,0])))
        else:
            self.vis[self.payload.shape].set_transform(
                    tf.translation_matrix(payloadSt[0:3]).dot(
                        tf.quaternion_matrix(payloadSt[3:7])))
            
        for name, quad in self.quads.items():
            self.vis[name].set_transform(
                    tf.translation_matrix(quad.state[0:3]).dot(
                        tf.quaternion_matrix(quad.state[3:7])))
            cablePos = np.linspace(payloadSt[0:3], quad.state[0:3], num=2).T
            cableMat  = g.LineBasicMaterial(linewidth=1, color=0x000000)
            self.vis["cable_"+name].set_object(g.Line(g.PointsGeometry(cablePos), material=cableMat))
        


def quad3dpayload_meshcatViewer():    
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, help="robot model: quad3dpayload, (point rigid: for n robots)")
    parser.add_argument('--env', type=str, help="environment")
    parser.add_argument('--result', type=str, help="result trajectory")
    parser.add_argument("-i", "--interactive", action="store_true")  # on/off flag
    
    args   = parser.parse_args()
    pathtoenv = args.env
    pathtoresult = args.result
    robotname = args.robot
    with open(pathtoenv, "r") as file:
        env = yaml.safe_load(file)
    with open(pathtoresult, 'r') as file:
        path = yaml.safe_load(file)
    
    pType   = robotname 
    quadNum = env["robots"][0]["quadsNum"]
    l       =  env["robots"][0]["l"]
    start   = env["robots"][0]["start"]
    goal     = env["robots"][0]["goal"]
    obstacles = env["environment"]["obstacles"]
    quadsPayload = QuadPayloadRobot(quadNum=quadNum, pType=pType)
    states = path['result']['states']

    visualizer = Visualizer(quadsPayload, obstacles)
    if args.interactive == True:     
        visualizer.vis.open()
    while True: 
        for state in states:
            visualizer.updateVis(state)
            time.sleep(0.01)
        
if __name__ == "__main__":
    main()