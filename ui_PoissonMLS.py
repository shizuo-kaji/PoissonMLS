# -*- coding: utf-8 -*-

#  User interface for Poisson MLS Deformer plugin
#  The last object in the selected ones will be the target while the others will serve as control points

#  @author      Shizuo KAJI
#  @date        2016/11/14

# for debug
#import debugmaya
#debugmaya.startDebug()

# Import Maya Modules
import maya.cmds as cmds
import pymel.core as pm

deformerTypes = ["PoissonMLS"]

for type in deformerTypes:
    try:
        cmds.loadPlugin(type)
    except:
        print("Plugin %s already loaded" %(type))

## prepare interface
class UI_PoissonMLS:
    uiID = "PoissonMLS"
    title = "PoissonMLS"
    deformers = []
    probes = {}

    ## Constructor
    def __init__(self):
        if pm.window(self.uiID, exists=True):
            pm.deleteUI(self.uiID)
        win = pm.window(self.uiID, title=self.title, menuBar=True)
        with win:
            pm.menu( label='Create', tearOff=True )
            for type in deformerTypes:
                pm.menuItem( label=type, c=pm.Callback( self.initPlugin, type) )
            self._parentLayout = pm.columnLayout( adj=True )
            with self._parentLayout:
                self.createUISet()

    def createUISet(self):
        self._childLayout = pm.columnLayout( adj=True )
        with self._childLayout:
            self.deformers = [pm.ls(type=deformerTypes[i]) for i in range(len(deformerTypes))]
            for i in range(len(deformerTypes)):
                for node in self.deformers[i]:
                    self.probes[node] = pm.listConnections(node.cp)
            # specific
            for node in self.deformers[0]:
                frameLayout = pm.frameLayout( label=node.name(), collapsable = True)
                with frameLayout:
                    self.createCommonAttr(node, deformerTypes[0])
                    indices = cmds.getAttr(node+".cp", multiIndices=True)
                    if indices:
                        for j in indices:
                            with pm.rowLayout(numberOfColumns=1) :
                                pm.attrFieldSliderGrp(label=node.ctlw[j].getAlias(), min=0, max=10.0, attribute=node.ctlw[j])

    # create deformer node and connection
    def initPlugin(self, deformerType):
        # get transform nodes for the selected objects
        transforms = pm.selected(tr=1)
        if not transforms:
            return
        pm.select( transforms[-1])       # the deformer is attached to the last selected object
        node = pm.ls(cmds.deformer(type=deformerType)[0])[0]
        cmds.makePaintable(deformerType, 'weights', attrType='multiFloat', shapeMode='deformer')
        if len(transforms)>1:
            self.addProbe(node,deformerType,transforms[:-1])
        self.updateUI()

    # add selected transform as a new probe
    def addProbe(self,node,deformerType,newProbes):
        indexes = cmds.getAttr(node+".cp", multiIndices=True)
        if not indexes:
            n=0
        else:
            n=indexes[-1]+1
        # connect pm first to avoid unnecessary arap computations
        for j in range(len(newProbes)):
            cmds.connectAttr(newProbes[j]+".center", node+".cp[%s]" %(j+n))
            cmds.connectAttr(newProbes[j]+".scaleX", node+".ctlw[%s]" %(j+n))
            node.icp[j+n].set(node.cp[j+n].get())

    # add selected transform as a new probe
    def addSelectedProbe(self,node,deformerType):
        newProbes = pm.selected(tr=1)
        self.addProbe(node,deformerType,newProbes)
        self.updateUI()

    # delete deformer node
    def deleteNode(self,node):
        cmds.delete(node.name())
        self.updateUI()

    # redraw UI
    def updateUI(self):
        pm.deleteUI( self._childLayout )
        pm.setParent(self._parentLayout)
        self.createUISet()

    def createCommonAttr(self,node,deformerType):
        with pm.rowLayout(numberOfColumns=3) :
            pm.button( l="Add selection to ctrl points", c=pm.Callback( self.addSelectedProbe, node, deformerType) )
            pm.button( l="Delete deformer", c=pm.Callback( self.deleteNode, node))
            pm.attrControlGrp( label="Poisson", attribute= node.poisson)
        with pm.rowLayout(numberOfColumns=4) :
            pm.attrControlGrp( label="MLS mode", attribute= node.mlsm)
            pm.attrControlGrp( label="area weight", attribute= node.aw)
            pm.attrControlGrp( label="neighbour weighting", attribute= node.nghbrw)
            pm.attrFieldSliderGrp( label="iteration", min=1, max=20, attribute=node.it)
        with pm.rowLayout(numberOfColumns=4) :
            pm.attrControlGrp( label="Weight mode", attribute= node.wtm)
            pm.attrFieldSliderGrp(label="effect radius", min=0.001, max=20.0, attribute=node.er)
            pm.attrControlGrp( label="normalise weight", attribute= node.nw)
            pm.attrControlGrp( label="normExponent", attribute=node.ne)
        with pm.rowLayout(numberOfColumns=4) :
            pm.attrControlGrp( label="tet mode", attribute= node.tm)
            pm.attrControlGrp( label="constraint mode", attribute= node.ctm)
            pm.attrFieldSliderGrp( label="constraint weight", min=0.001, max=1000, attribute=node.cw)
            pm.attrFieldSliderGrp(label="constraint radius", min=0.001, max=10.0, attribute=node.cr)

