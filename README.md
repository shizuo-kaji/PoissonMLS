PoissonMLS plugin for Maya
/**
 * @brief Poisson MLS Deformer plugins for Maya
 * @section LICENSE The MIT License
 * @section requirements:  Eigen 3:  http://eigen.tuxfamily.org/
 * @section Autodesk Maya: http://www.autodesk.com/products/autodesk-maya/overview
 * @section (included) AffineLib: https://github.com/shizuo-kaji/AffineLib
 * @version 0.10
 * @date  19/Nov/2016
 * @author Shizuo KAJI
 */

## The plugins
This is a deformer plugin based on the Moving Least Square deformation
and the Poisson mesh editing techniques.
The algorithm is presented at MEIS2016:

http://www.skaji.org/files/MEIS2016-MLS.pdf


## How to compile:
- Mac OS: Look at the included Xcode project file
- Windows: Look at the included Visual Studio project. __DO NOT__ turn on AVX instructions.
- Other: Look at the included Makefile
- on some systems, specifying the compiler option -DEIGEN_DONT_VECTORIZE may be necessary to avoid compilation errors (thank giordi91 for this information)


## How to use:
- put the plugins in "MAYA_PLUG_IN_PATH"
- put the UI python script in "MAYA_SCRIPT_PATH"
- open script editor in Maya and type in the following Python command:

```python
import ui_PoissonMLS as ui
ui.UI_PoissonMLS()
```

- Select multiple objects which you would like to make control points.
- While holding the shift key, select the target mesh to be deformed.
(that is, the target mesh is the last among the selected objects.)
- Create deformer node from the menu of the included UI script.
- By default, the weight(influence) of each control point is connected by the scale attribute.

## LIMITATION:
The "poisson" option works only with clean meshes.
First apply "Cleanup" from "Mesh" menu
to remove zero faces and edges, non-manifold geometry, etc.
