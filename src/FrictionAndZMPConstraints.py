import numpy as np
import random
import FrictionLimits
"""
FrictionAndZMPConstraints.py

Variables:
configlist  -- [q, qd, qdd]
bottledim   -- [dx, dy, bottleh] half width, half depth, and half height of the bottle

Convention: The bottle is a seperate body from the robot. Assume that
the robot must grab the bottle such that robot.GetGrabbed()[0] ==
bottle.
"""

X = np.array([1., 0., 0.])
Y = np.array([0., 1., 0.])
Z = np.array([0., 0., 1.])

DELTA = 1e-8
RNG = random.SystemRandom()


def ComputeInteractionForces(robot, configlist):
    """
    ComputeInteractionForces computes friction and normal forces
    between the tray and the bottle.
    The function returns a list [f, N].
    """
    # Retrieve variables
    g = robot.GetEnv().GetPhysicsEngine().GetGravity()    
    q = configlist[0]
    qd = configlist[1]
    qdd = configlist[2]

    tray = robot.GetLink('tray')
    bottle = robot.GetGrabbed()[0].GetLink('bottle_lowerpart')
    mb = bottle.GetMass()
    
    with robot:
        robot.SetDOFValues(q)
        robot.SetDOFVelocities(qd)
        
        pt = tray.GetGlobalCOM()
        pb = bottle.GetGlobalCOM()
        # Acceleration of the tray
        acc = robot.GetLinkAccelerations(qdd)[tray.GetIndex()]
        at = np.array(acc[0:3])      # linear
        alpha = np.array(acc[3:6])   # angular
        # Angular velocity of the tray (and thus of the bottle)
        w = robot.GetLinkVelocities()[tray.GetIndex()][3:6]
        
        # Linear acceleration of the bottle
        ab = at + np.cross(w, np.cross(w, pb - pt)) + np.cross(alpha, pb - pt)

        # tray's local Z
        nz = GetLocalVector(tray, Z)
        # Normal force
        N = np.dot(nz, mb*(ab - g))
        # Friction force
        f = mb*(ab - g) - N*nz       

    return [f, N]
        

def ZMP(robot, configlist, bottledim):
    # Retrieve variables
    h = bottledim[2]
    g = robot.GetEnv().GetPhysicsEngine().GetGravity()    
    q = configlist[0]
    qd = configlist[1]
    qdd = configlist[2]

    tray = robot.GetLink('tray')
    bottle = robot.GetGrabbed()[0].GetLink('bottle_lowerpart')
    mb = bottle.GetMass()    

    with robot:
        robot.SetDOFValues(q)
        robot.SetDOFVelocities(qd)

        pt = tray.GetGlobalCOM()
        pb = bottle.GetGlobalCOM()
        # Acceleration of the tray
        acc = robot.GetLinkAccelerations(qdd)[tray.GetIndex()]
        at = np.array(acc[0:3])      # linear
        alpha = np.array(acc[3:6])   # angular
        # Angular velocity of the tray (and thus of the bottle)
        w = robot.GetLinkVelocities()[tray.GetIndex()][3:6]

        # Linear acceleration of the bottle
        ab = at + np.cross(w, np.cross(w, pb - pt)) + np.cross(alpha, pb - pt)
        
        # Inertia matrix of the bottle in the global frame
        Rb = bottle.GetTransform()[0:3, 0:3]
        localIb = bottle.GetLocalInertia() # local
        Ib = np.dot(Rb, np.dot(localIb, Rb.T))
        
        # tray's local Z
        nz = GetLocalVector(tray, Z)
        
        # The derivative of the bottle's angular momentum
        M = np.dot(Ib, alpha) + np.cross(w, np.dot(Ib, w))
        # Normal force
        N = np.dot(nz, mb*(ab - g))
        # Global location of the ZMP
        D = pb - (1.0/N)*(h*mb*(ab - g) + np.cross(nz, M))

    return D


def RelativeZMP(robot, configlist, bottledim):
    """
    RelativeZMP returns the position of the ZMP described in the
    tray's frame.
    """
    # Retrieve variables
    [dx, dy, h] = bottledim
    q = configlist[0]
    qd = configlist[1]
    qdd = configlist[2]
    bottle = robot.GetGrabbed()[0].GetLink('bottle_lowerpart')

    relzmp = np.zeros(2)
    zmp = ZMP(robot, configlist, bottledim)
    with robot:
        robot.SetDOFValues(q)
        pb = bottle.GetGlobalCOM()
        nx = GetLocalVector(bottle, X)
        ny = GetLocalVector(bottle, Y)
        nz = GetLocalVector(bottle, Z)

        # Projection of the bottle's COM on the tray
        center = pb - h*nz
        relzmp[0] = np.dot(nx, zmp - center)
        relzmp[1] = np.dot(ny, zmp - center)

    return relzmp


def COMProjection(robot, q, bottledim):
    configlist = [q, np.zeros(len(q)), np.zeros(len(q))]
    return ZMP(robot, configlist, bottledim)


def RelativeCOMProjection(robot, q, bottledim):
    configlist = [q, np.zeros(len(q)), np.zeros(len(q))]
    return RelativeZMP(robot, configlist, bottledim)


def GetLocalVector(link, vect):
    """
    GetLocalVector returns the local vector described in the global
    frame. For example, if vect := Z, then this function returns the
    link's Z-axis described in the global frame.
    """
    R = link.GetTransform()[0:3, 0:3]
    return np.dot(R, vect) ## a vector vect in the link's frame
 

def CheckConstraintsForConfiguration(robot, configlist, bottledim, mu):
    q = configlist[0]
    bottle = robot.GetGrabbed()[0].GetLink('bottle_lowerpart')
    tray = robot.GetLink('tray')

    [f, N] = ComputeInteractionForces(robot, configlist)
    with robot:
        robot.SetDOFValues(q)
        fx = np.dot(f, GetLocalVector(bottle, X))
        fy = np.dot(f, GetLocalVector(bottle, Y))

    # Check friction constraints
    if (abs(fx) > mu*N/np.sqrt(2)) or (abs(fy) > mu*N/np.sqrt(2)):
        return False
    
    # Check ZMP constraints
    [dx, dy, h] = bottledim
    relzmp = RelativeZMP(robot, configlist, bottledim)
    if (abs(relzmp[0]) > dx) or (abs(relzmp[1]) > dy):
        return False

    return True


# For computing A and b such that A qdd <= b describe the set of
# possible accelerations.
def NextControlConstraints(robot, configlist, bottledim, mu):
    # Retrieve variables
    g = robot.GetEnv().GetPhysicsEngine().GetGravity()
    [dx, dy, h] = bottledim
    q = configlist[0]
    qd = configlist[1]
    qdd = configlist[2] # previous control
    qddBounds = robot.GetDOFAccelerationLimits()
    tray = robot.GetLink('tray')
    bottle = robot.GetGrabbed()[0].GetLink('bottle_lowerpart')
    nlink = len(robot.GetLinks())
    bottlelinkindex = nlink - 1
    m = bottle.GetMass()

    u2 = mu/np.sqrt(2)

    with robot:
        robot.SetDOFValues(q)
        robot.SetDOFVelocities(qd)
        pt = tray.GetGlobalCOM()
        pb = bottle.GetGlobalCOM()
        
        # Jacobian
        Jv = robot.CalculateJacobian(bottlelinkindex, pb)
        Jw = robot.CalculateAngularVelocityJacobian(bottlelinkindex)
        
        # Acceleration of the tray
        acc = robot.GetLinkAccelerations(qdd)[tray.GetIndex()]
        at = np.array(acc[0:3])      # linear
        alpha = np.array(acc[3:6])   # angular
        # Angular velocity of the tray (and thus of the bottle)
        w = robot.GetLinkVelocities()[tray.GetIndex()][3:6]

        # Linear acceleration of the bottle
        ab = at + np.cross(w, np.cross(w, pb - pt)) + np.cross(alpha, pb - pt)
        
        # Inertia matrix of the bottle in the global frame
        Rb = bottle.GetTransform()[0:3, 0:3]
        localIb = bottle.GetLocalInertia() # local
        Ib = np.dot(Rb, np.dot(localIb, Rb.T))

        nx = GetLocalVector(tray, X)
        ny = GetLocalVector(tray, Y)
        nz = GetLocalVector(tray, Z)

        # Compute derivatives of Jacobians
        qd_prev = qd - DELTA*qdd
        q_prev = q - DELTA*qd_prev - 0.5*DELTA*DELTA*qdd
        robot.SetDOFValues(q_prev)
        robot.SetDOFVelocities(qd_prev)
        pt_prev = tray.GetGlobalCOM()
        pb_prev = bottle.GetGlobalCOM()
        
        # Jacobian_prev
        Jv_prev = robot.CalculateJacobian(bottlelinkindex, pb_prev)
        Jw_prev = robot.CalculateAngularVelocityJacobian(bottlelinkindex)
        
        # Backward difference (following FrictionLimits.cpp in TOPP)
        norm_qd = np.linalg.norm(qd)
        Jvd = (norm_qd/DELTA)*(Jv - Jv_prev)
        Jwd = (norm_qd/DELTA)*(Jw - Jw_prev)
        print "Jvd", Jvd

    # Friction constraints
    A1 = np.vstack((-m*nz,
                    -u2*nz + nx,
                    -u2*nz - nx,
                    -u2*nz + ny,
                    -u2*nz - ny))

    temp1 = g - np.dot(Jvd, qd)
    
    # Af qdd <= bf
    Af = np.dot(A1, Jv)
    bf = np.dot(A1, temp1)
    return (Af, bf)
"""
    # ZMP constraints
    # Temporary variables
    IJw = np.dot(Ib, Jw)
    IJwdqd = np.dot(Ib, np.dot(Jwd, qd))
    wIw = np.cross(w, np.dot(Ib, w))
    
    A2 = np.vstack((-dx*nz + h*nx,
                    -dx*nz - h*nx,
                    -dy*nz + h*ny,
                    -dy*nz - h*ny))

    B2 = (1./m)*np.vstack((ny, -ny, -nx, nx))

    Az = np.dot(A2, Jv) - np.dot(B2, IJw)
    bz = np.dot(A2, temp1) + np.dot(B2, IJwdqd + wIw)

    A = np.vstack((Af, Az))
    b = np.hstack((bf, bz))
    # # Combined constraints
    # E = np.eye(len(qddBounds)) # Don't forget to add acceleration bounds
    # A = np.vstack((Af, Az, E, -E))
    # b = np.hstack((bf, bz, qddBounds, qddBounds))
    return (A, b)
"""

def SampleInPolygon(A, b, bounds, prevcontrol):
    """
    SampleInPolygon samples a vector x such that Ax <= b while x
    respects the bounds.

    bounds = [lower, upper]

    Now the implementation only uses rejection sampling. I will change
    it later when I find a better way of doing this.
    """
    
    assert(len(A) == len(b))
    x = np.zeros(len(A[0]))
    L = bounds[0]
    U = bounds[1]
    _i = 0
    while (((not (np.alltrue(np.dot(A, x) <= b))) or (np.linalg.norm(x) < 1e-6)) 
           and (_i <= 1000)):
        _i += 1
        for i in xrange(len(x)):
            x[i] = RNG.uniform(L[i], U[i])

    if not (np.alltrue(np.dot(A, x) <= b)):
        if (np.alltrue(np.dot(A, prevcontrol) <= b)):
            return prevcontrol
        else:
            # assert(False)
            return None
    return x
    

def DrawAcceleration(robot, configlist):
    tray = robot.GetLink('tray')
    bottle = robot.GetGrabbed()[0].GetLink('bottle_lowerpart')
    mb = bottle.GetMass()

    q = configlist[0]
    qd = configlist[1]
    qdd = configlist[2]
    
    robot.SetDOFValues(q)
    robot.SetDOFVelocities(qd)
    pt = tray.GetGlobalCOM()
    pb = bottle.GetGlobalCOM()

    # Acceleration of the tray
    acc = robot.GetLinkAccelerations(qdd)[tray.GetIndex()]
    at = np.array(acc[0:3])      # linear
    alpha = np.array(acc[3:6])   # angular
    # Angular velocity of the tray (and thus of the bottle)
    w = robot.GetLinkVelocities()[tray.GetIndex()][3:6]

    # Linear acceleration of the bottle
    ab = at + np.cross(w, np.cross(w, pb - pt)) + np.cross(alpha, pb - pt)

    return DrawLine(robot, pb, pb + ab)


def DrawZMP(robot, configlist, bottledim):
    q = configlist[0]
    qd = configlist[1]
    qdd = configlist[2]
    bottle = robot.GetGrabbed()[0].GetLink('bottle_lowerpart')
    h = bottledim[2]
    
    robot.SetDOFValues(q)
    zmp = ZMP(robot, configlist, bottledim)
    nz = GetLocalVector(bottle, 4.*h*Z)
    zmp_up = zmp + nz
    return DrawLine(robot, zmp, zmp_up)


def DrawLine(robot, p1, p2, COLORS = np.array([1., 0., 0.])):
    env = robot.GetEnv()
    handle = env.drawlinestrip(np.vstack((p1, p2)), linewidth = 3, colors = COLORS)
    return handle
