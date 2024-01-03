# -*- coding: utf-8 -*-

import Sofa.Core
from splib3.animation import AnimationManager
from params import *
from math import sin, cos, pi


class BeamController(Sofa.Core.Controller):

    def __init__(self, node):
        Sofa.Core.Controller.__init__(self)
        self.node = node
        self.dt = node.getDt()

        self.q1Init = None
        self.q2Init = None
        self.q3Init = None
        self.q4Init = None

        self.time = 0.0
        self.stabTime = 0.0
        self.positions = self.node.Model.MechanicalObject.position.value
        self.writeDisplacement("init,", "w")

        self.trajectory = trajectory
        self.convergenceWaiting = convergenceWaiting
        self.ptTrajectory = ptTrajectory
        self.nbPtTrajectory = nbPtTrajectory

        return

    def writeDisplacement(self, pre, openType):
        positions = self.node.Model.getMechanicalState().position.value
        wDisplacementPlatform1 = open(displacementPlatform1, openType)
        wDisplacementPlatform1.write(pre + str(positions[0]) + '\n')
        wDisplacementPlatform1.close()
        wDisplacementPlatform2 = open(displacementPlatform2, openType)
        wDisplacementPlatform2.write(pre + str(positions[1]) + '\n')
        wDisplacementPlatform2.close()
        wDisplacementFinger1 = open(displacementFinger1, openType)
        wDisplacementFinger1.write(pre + str(positions[0]) + '\n')
        wDisplacementFinger1.close()
        wDisplacementFinger2 = open(displacementFinger2, openType)
        wDisplacementFinger2.write(pre + str(positions[1]) + '\n')
        wDisplacementFinger2.close()

    def onAnimateBeginEvent(self, event):
        self.time += self.dt
        positions = self.node.Model.MechanicalObject.position

        if self.time > self.stabTime:  # the objective is to wait the simulation to stabilize (non-mandatory step)
            if self.convergenceWaiting > tolerance:
                if self.ptTrajectory < self.nbPtTrajectory:

                    if self.ptTrajectory == 0:
                        self.q1Init = self.positions[12][1]
                        self.q2Init = self.positions[7][0]
                        self.q3Init = self.positions[13][1]
                        self.q4Init = self.positions[6][0]

                    if self.ptTrajectory > 1:
                        # remove the force that allows to move out of the plan at the beginning of the simulation
                        self.node.Model.ConstantForceField.force = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                    # get data after stabilization
                    self.writeDisplacement(str(self.ptTrajectory), "a")

                    # new positions for tha actuators
                    positions[6][0] = self.q4Init + float(self.trajectory[self.ptTrajectory][3])
                    positions[7][0] = self.q2Init + float(self.trajectory[self.ptTrajectory][1])
                    positions[12][1] = self.q1Init + float(self.trajectory[self.ptTrajectory][0])
                    positions[13][1] = self.q3Init + float(self.trajectory[self.ptTrajectory][2])

                    self.node.Model.MechanicalObject.position = positions
                    # reinitialization of the parameters convergence and incrementation
                    self.convergenceWaiting = 0
                    self.ptTrajectory += 1

                elif self.ptTrajectory == self.nbPtTrajectory:

                    self.writeDisplacement(str(self.ptTrajectory), "a")
                    self.ptTrajectory += 1

            self.convergenceWaiting += 1


def createScene(rootNode):
    rootNode.gravity.value = [0, 0, 0]
    rootNode.addObject("BackgroundSetting", color=[1, 1, 1, 1])

    settings = rootNode.addChild("Settings")
    settings.addObject('RequiredPlugin', name='SoftRobots')
    settings.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    settings.addObject('RequiredPlugin', name='BeamAdapter')
    settings.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')
    settings.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')
    settings.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')
    settings.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')
    settings.addObject('RequiredPlugin', name='Sofa.Component.Mass')
    settings.addObject('RequiredPlugin', name='Sofa.Component.MechanicalLoad')
    settings.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')
    settings.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')
    settings.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')
    settings.addObject('RequiredPlugin', name='Sofa.Component.Visual')
    settings.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
    settings.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
    settings.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')
    settings.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')

    rootNode.addObject(AnimationManager(rootNode))

    rootNode.addObject('VisualStyle', displayFlags='showVisualModels')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver')

    ########################################
    # Robot model                          #
    ########################################
    model = rootNode.addChild('Model')
    model.addObject('EulerImplicitSolver', rayleighStiffness=0.02, rayleighMass=0.01)
    model.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
    model.addObject('GenericConstraintCorrection')
    model.addObject('MeshTopology', edges=[[0, 1], [0, 1], [2, 0],
                                           [3, 0], [6, 2], [6, 3],
                                           [1, 4], [1, 5], [4, 7],
                                           [5, 7], [0, 8], [1, 10],
                                           [0, 9], [1, 11], [8, 12],
                                           [10, 12], [9, 13], [11, 13]])
    model.addObject('MechanicalObject', template='Rigid3d', showObject=True,
                    position=[[-1.45, 0, 0, 0, 0, 0, 1],  # localisation des reperes de chacun des elements rigides
                              [1.45, 0, 0, 0, 0, 0, 1],
                              [-5.6, 0.7, 0, 0, 0, 0, 1],
                              [-5.6, -0.7, 0, 0, 0, 0, 1],
                              [5.6, 0.7, 0, 0, 0, 0, 1],
                              [5.6, -0.7, 0, 0, 0, 0, 1],
                              [-9.75, 0.0, 0, 0, 0, 0, 1],
                              [9.75, 0.0, 0, 0, 0, 0, 1],
                              [-1.45, 4.15, 0, 0, 0, 0, 1],
                              [-1.45, -4.15, 0, 0, 0, 0, 1],
                              [1.45, 4.15, 0, 0, 0, 0, 1],
                              [1.45, -4.15, 0, 0, 0, 0, 1],
                              [0, 8.3, 0, 0, 0, 0, 1],
                              [0, -8.3, 0, 0, 0, 0, 1]])

    model.addObject('BeamInterpolation', name='interpolation', crossSectionShape='rectangular', lengthZ=0.4,
                    lengthY=0.4,  # creation des beams entre les elements rigide en silicium
                    dofsAndBeamsAligned=False, defaultYoungModulus=1.4e5,
                    DOF0TransformNode0=[
                        [1.25, 0.7, 0, 0, 0, 0, 1],
                        [1.25, -0.7, 0, 0, 0, 0, 1],
                        [2.5, 0, 0, 0, 0, 0, 1],
                        [2.5, 0, 0, 0, 0, 0, 1],
                        [1.25, 0.7, 0, 0, 0, 0, 1],
                        [1.25, -0.7, 0, 0, 0, 0, 1],
                        [1.25, 0.7, 0, 0, 0, 0, 1],
                        [1.25, -0.7, 0, 0, 0, 0, 1],
                        [2.5, 0, 0, 0, 0, 0, 1],
                        [2.5, 0, 0, 0, 0, 0, 1],
                        [0, 1.25, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                        [0, 1.25, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                        [0, -1.25, -0.0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                        [0, -1.25, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                        [0, 2.5, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                        [0, 2.5, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                        [0, -2.5, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                        [0, -2.5, 0, 0, 0, -sin(pi / 4), cos(pi / 4)]
                    ],

                    DOF1TransformNode1=[
                        [-1.25, 0.7, 0, 0, 0, 0, 1],
                        [-1.25, -0.7, 0, 0, 0, 0, 1],
                        [-1.25, 0.7, -0.0, 0, 0, 0, 1],
                        [-1.25, -0.7, -0.0, 0, 0, 0, 1],
                        [-2.5, 0, 0, 0, 0, 0, 1],
                        [-2.5, 0, 0, 0, 0, 0, 1],
                        [-2.5, 0, 0, 0, 0, 0, 1],
                        [-2.5, 0, 0, 0, 0, 0, 1],
                        [-1.25, 0.7, 0, 0, 0, 0, 1],
                        [-1.25, -0.7, 0, 0, 0, 0, 1],
                        [0, -2.5, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                        [0, -2.5, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                        [0, 2.5, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                        [0, 2.5, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                        [-1.45, -1.25, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                        [1.45, -1.25, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                        [-1.45, 1.25, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                        [1.45, 1.25, 0, 0, 0, -sin(pi / 4), cos(pi / 4)]
                    ]
                    )

    model.addObject('BeamInterpolation', name='interpolation2', crossSectionShape='rectangular', lengthZ=0.4,
                    lengthY=0.6,  # creation des beams entre les elements rigides en silicium
                    dofsAndBeamsAligned=False, defaultYoungModulus=1.4e5,
                    DOF0TransformNode0=[[1.25, 0.7, 0, 0, 0, 0, 1],
                                        [1.25, -0.7, 0, 0, 0, 0, 1],
                                        [2.5, 0, 0, 0, 0, 0, 1],
                                        [2.5, 0, 0, 0, 0, 0, 1],
                                        [1.25, 0.7, 0, 0, 0, 0, 1],
                                        [1.25, -0.7, 0, 0, 0, 0, 1],
                                        [1.25, 0.7, 0, 0, 0, 0, 1],
                                        [1.25, -0.7, 0, 0, 0, 0, 1],
                                        [2.5, 0, 0, 0, 0, 0, 1],
                                        [2.5, 0, 0, 0, 0, 0, 1],
                                        [0, 1.25 - 0.0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                        # orientation des beams avec les quaternions
                                        [0, 1.25, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                        [0, -1.25, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                                        [0, -1.25, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                                        [0, 2.5, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                        [0, 2.5, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                        [0, -2.5, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                                        [0, -2.5, 0, 0, 0, -sin(pi / 4), cos(pi / 4)]],

                    DOF1TransformNode1=[[-1.25, 0.7, 0, 0, 0, 0, 1],
                                        [-1.25, -0.7, 0, 0, 0, 0, 1],
                                        [-1.25, 0.7, -0.0, 0, 0, 0, 1],
                                        [-1.25 - 0.7, -0.0, 0, 0, 0, 1],
                                        [-2.5, 0, 0, 0, 0, 0, 1],
                                        [-2.5, 0, 0, 0, 0, 0, 1],
                                        [-2.5, 0, 0, 0, 0, 0, 1],
                                        [-2.5, 0, 0, 0, 0, 0, 1],
                                        [-1.25, 0.7, 0, 0, 0, 0, 1],
                                        [-1.25, -0.7, 0, 0, 0, 0, 1],
                                        [0, -2.5, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                        [0, -2.5, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                        [0, 2.5, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                                        [0, 2.5, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                                        [-1.45, -1.25, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                        [1.45, -1.25, 0, 0, 0, sin(pi / 4), cos(pi / 4)],
                                        [-1.45, 1.25, 0, 0, 0, -sin(pi / 4), cos(pi / 4)],
                                        [1.45, 1.25, 0, 0, 0, -sin(pi / 4), cos(pi / 4)]
                                        ]
                    )

    # creation des masses des differents elements en gramme
    model.addObject('UniformMass', name='pince0', indices=[0], totalMass=[0.0043748])
    model.addObject('UniformMass', name='pince1', indices=[1], totalMass=[0.0043748])
    model.addObject('UniformMass', name='leg1', indices=[2], totalMass=[0.00283836])
    model.addObject('UniformMass', name='leg2', indices=[3], totalMass=[0.00283836])
    model.addObject('UniformMass', name='leg3', indices=[4], totalMass=[0.00283836])
    model.addObject('UniformMass', name='leg4', indices=[5], totalMass=[0.00283836])
    model.addObject('UniformMass', name='leg5', indices=[8], totalMass=[0.00283836])
    model.addObject('UniformMass', name='leg6', indices=[9], totalMass=[0.00283836])
    model.addObject('UniformMass', name='leg7', indices=[10], totalMass=[0.00283836])
    model.addObject('UniformMass', name='leg8', indices=[11], totalMass=[0.00283836])
    model.addObject('UniformMass', name='base1', indices=[6], totalMass=[0.0099042])
    model.addObject('UniformMass', name='base2', indices=[7], totalMass=[0.0099042])
    model.addObject('UniformMass', name='base3', indices=[12], totalMass=[0.0099042])
    model.addObject('UniformMass', name='base4', indices=[13], totalMass=[0.0099042])

    model.addObject('FixedProjectiveConstraint',
                    indices=[6, 7, 12, 13])  # fixe les deplacements des actionneurs suivant les directions voulues
    model.addObject('ConstantForceField',
                    indices=[0, 1],
                    forces=[0.0, 0.0, 1, 0.0, 0.0, 0.0],
                    showArrowSize=0.1)

    effector = model.addChild('Effector')
    effector.addObject('MechanicalObject', position=[[1.43, 0, 2.7], [-1.43, 0, 2.7]])
    effector.addObject('RigidMapping', rigidIndexPerPoint=[0, 1])
    effector.addObject('SphereCollisionModel', radius=0.01)

    model.addObject('AdaptiveBeamForceFieldAndMass', massDensity=0.97e-6, interpolation='@interpolation')

    ########################################
    # Robot Visual                         #
    ########################################
    visu0 = model.addChild('VisuLeftPlatform')
    visu0.addObject('MeshSTLLoader', filename='data/platformFinger.stl', translation=[1.45, 1.25, -2.5],
                    rotation=[0, 0, 180])
    visu0.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu0.addObject('RigidMapping', index=0)

    visu1 = model.addChild('VisuRightPlatform')
    visu1.addObject('MeshSTLLoader', filename='data/platformFinger.stl', translation=[-1.45, -1.25, -2.5])
    visu1.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu1.addObject('RigidMapping', index=1)

    visu2 = model.addChild('VisuLeg1')
    visu2.addObject('MeshSTLLoader', filename='data/leg.stl', translation=[-1.5, 0.2, -0.2], rotation=[90, 0, 0])
    visu2.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu2.addObject('RigidMapping', index=2)

    visu3 = model.addChild('Visuleg2')
    visu3.addObject('MeshSTLLoader', filename='data/leg.stl', translation=[-1.5, 0.2, -0.2], rotation=[90, 0, 0])
    visu3.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu3.addObject('RigidMapping', index=3)

    visu4 = model.addChild('Visuleg3')
    visu4.addObject('MeshSTLLoader', filename='data/leg.stl', translation=[-1.5, 0.2, -0.2], rotation=[90, 0, 0])
    visu4.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu4.addObject('RigidMapping', index=4)

    visu5 = model.addChild('Visuleg4')
    visu5.addObject('MeshSTLLoader', filename='data/leg.stl', translation=[-1.5, 0.2, -0.2], rotation=[90, 0, 0])
    visu5.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu5.addObject('RigidMapping', index=5)

    visu6 = model.addChild('VisuBaseXNeg')
    visu6.addObject('MeshSTLLoader', filename='data/base.stl', translation=[1.25, -1.75, -0.2], rotation=[0, 0, 90])
    visu6.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu6.addObject('RigidMapping', index=6)

    visu7 = model.addChild('VisuBaseXPos')
    visu7.addObject('MeshSTLLoader', filename='data/base.stl', translation=[1.25, -1.75, -0.2], rotation=[0, 0, 90])
    visu7.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu7.addObject('RigidMapping', index=7)

    visu8 = model.addChild('VisuLeg5')
    visu8.addObject('MeshSTLLoader', filename='data/leg.stl', translation=[-0.2, -1.5, -0.2], rotation=[90, 0, 90])
    visu8.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu8.addObject('RigidMapping', index=8)

    visu9 = model.addChild('VisuLeg6')
    visu9.addObject('MeshSTLLoader', filename='data/leg.stl', translation=[-0.2, -1.5, -0.2], rotation=[90, 0, 90])
    visu9.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu9.addObject('RigidMapping', index=9)

    visu10 = model.addChild('VisuLeg7')
    visu10.addObject('MeshSTLLoader', filename='data/leg.stl', translation=[-0.2, -1.5, -0.2], rotation=[90, 0, 90])
    visu10.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu10.addObject('RigidMapping', index=10)

    visu11 = model.addChild('VisuLeg8')
    visu11.addObject('MeshSTLLoader', filename='data/leg.stl', translation=[-0.2, -1.5, -0.2], rotation=[90, 0, 90])
    visu11.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu11.addObject('RigidMapping', index=11)

    visu12 = model.addChild('VisuBaseYNeg')
    visu12.addObject('MeshSTLLoader', filename='data/baseLarge2.stl', translation=[-2.25, -1.25, -0.2])
    visu12.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu12.addObject('RigidMapping', index=12)

    visu13 = model.addChild('VisuBaseYPos')
    visu13.addObject('MeshSTLLoader', filename='data/baseLarge2.stl', translation=[-2.25, -1.25, -0.2])
    visu13.addObject('OglModel', src='@MeshSTLLoader', name='visu')
    visu13.addObject('RigidMapping', index=13)

    rootNode.addObject(BeamController(rootNode))

    return rootNode
