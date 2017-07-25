import graphics.colorNames as colorNames
reload(colorNames)
import geometry.shapes as shapes

### Mathematica interface

closeupView = 'ViewAngle -> 0.3, ViewPoint -> {2, 0, 1.0}'
closeupLights = 'Lighting -> {{"Directional", White, {{4, 0, 2}, {0.75,0.5,0.5}}}, {"Directional", White, {{4.5, -9, 6.0}, {0.75,0.5,0.5}}}}, Background -> LightBlue'

def mathMovie(displays, filenameOut = "./mathMovie",
              view = "ViewPoint -> {0, -1, 2}",
              lights = 'Lighting -> {{"Point", Red, {3.,0.,3.}}, {"Point", White, {-2.,-2.,3.}}, {"Point", Green, {3.,3.,3.}}, {"Point", Blue, {0.,3.,3.}}}',
              init = 0):
    if displays is None: return
    f = open(filenameOut, 'w')
    data = []
    this = []
    for i, entry in enumerate(displays):
        if entry[0] in ('clear', 'pause'):
            if this:
                data.append(this)
                this = []
            continue
        this.append(entry)
    if this: data.append(this)
    f.write('{')
    n = len(data)
    for i in range(init, n):
        if data[i]:
            f.write(toMathList(data[i], view, lights))
            if i < n-1 : f.write(',\n')
    f.write('}')
    f.close()
    print "Movie", filenameOut, "saved"
    return 

# The displays are (object, color, opacity)
def mathFile(displays,
             filenameOut = "./mathFig.mat",
             view = "ViewPoint -> {0, -1, 2}",
             lights = 'Lighting -> {{"Point", Red, {3.,0.,3.}}, {"Point", White, {-2.,-2.,3.}}, {"Point", Green, {3.,3.,3.}}, {"Point", Blue, {0.,3.,3.}}}'):
    f = open(filenameOut, 'w')
    f.write('{')
    f.write(toMathList(displays, view, lights))
    f.write(',\n')
    f.write('}')
    f.close()
    print "File", filenameOut, "saved"
    return 

def toMathList(pl, view, lights):
    strs = ''.join([toMath(p) for p in pl])
    return 'Graphics3D[{ %s }, ImageSize -> 1000, Boxed -> False, SphericalRegion -> True, %s, %s]'%(strs[:-1], view, lights)

def toMath(p):
    if p[0] in ('clear', 'pause'):
        return ''
    obj, color, opacity, time = p
    expr = ''
    if isinstance(obj, (shapes.Thing, shapes.Prim, shapes.Shape)):
        for o in obj.toPrims():
            expr += '{EdgeForm[],'+toMathColor(color)+','+toMathOpacity(opacity)+','+ \
                    toMathPoly3D(o)+'},'
    return expr

def toMathPoly3D(obj):
    return 'GraphicsComplex[ %s, Polygon[%s] ]'%(toMathVerts(obj), toMathFaces(obj))

def toMathVerts(obj):
    vstrs = ''.join(['{%6.4f, %6.4f, %6.4f},'%tuple(v.tolist()[:3]) for v in obj.vertices().T])
    return '{ %s }'% vstrs[:-1]

def toMathFaces(obj):
    fstrs = ''.join(['{ %s },'%','.join([str(i+1) for i in face.tolist()])\
                     for face in obj.faces()])
    return '{ %s }'% fstrs[:-1]

def toMathColor(color):
    colors = colorNames.colors
    def colorList(c):
        if isinstance(c, str):
            if c[0] == '#':
                return map(float.fromhex, [c[1:3], c[3:5], c[5:7]])
            else:
                return [x/256.0 for x in colors[c]]
        elif isinstance(c, (tuple, list)):
            return [x/256.0 for x in c]
        elif c is None:
            return [x/256.0 for x in colors['white']]
        else:
            return c
    return 'RGBColor[%4.2f, %4.2f, %4.2f]'%tuple(colorList(color))

def toMathOpacity(op):
    if not(isinstance(op, (int, float)) and 0.0 < op < 1.0):
        op = 1.0
    return 'Opacity[%s]'%str(op)

# f = OpenRead["/Users/tlp/ltpy/pr2_hpn/branches/v2/mathFig.mat"]
# ListAnimate[Read[f], AnimationRunning -> False, AnimationRepetitions -> 1]
