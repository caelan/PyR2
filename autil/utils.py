import numpy as np

# If equiv is true, don't map two objects that should be different
# into the same string

# if eq = True, use 6 digits otherwise 3

eqDig = 3 # was 8
nonEqDig = 3

def prettyString(struct, eq = True):
    dig = eqDig if eq else nonEqDig
    if type(struct) == list:
        return '[' + ', '.join([prettyString(item, eq) for item in struct]) +']'
    elif type(struct) == tuple:
        return '(' + ', '.join([prettyString(item,eq) for item in struct]) + ')'
    elif type(struct) == dict:
        return '{' + ', '.join([str(item)+':'+ prettyString(struct[item], eq) \
                                             for item in struct]) + '}'
    elif isinstance(struct, np.ndarray):
        # Could make this prettier...
        return str(struct)
    elif type(struct)!= int and type(struct) != bool and \
             hasattr(struct, '__float__'):
        struct = round(struct, dig)
        if struct == 0: struct = 0      #  catch stupid -0.0
        #return ("%5.8f" % struct) if eq else ("%5.3f" % struct)
        return ("%5."+str(dig)+"f") % struct
    elif hasattr(struct, 'getStr'):
        return struct.getStr(eq)
    elif hasattr(struct, 'prettyString'):
        return struct.prettyString(eq)
    else:
        return str(struct)