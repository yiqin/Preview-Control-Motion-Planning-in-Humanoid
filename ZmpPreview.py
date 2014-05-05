"""
  Module to implement a ZMP preview controller.

  This module is based on the papers:

  [1] Kajita, Shuuji, et al. "Biped walking pattern generation by
      using preview control of zero-moment point." Proc. IEEE Int'l
      Conf. on Robotics and Automation (ICRA), IEEE 2003.

  [2] Park, Jonghoon, and Youngil Youm. "General ZMP preview control
      for bipedal walking." Proc. IEEE Int'l Conf. on Robotics and
      Automation (ICRA), IEEE 2007

"""

import numpy as np
import scipy
import scipy.linalg
import sys

class ZmpPreview:

    """Class to encapsulate the ZMP preview controller. Begin by
    initializing the controller with information about timestep, CoM
    height, etc.  Then initialize a stateErr object using
    initStateErr(). Finally, you can run the controller by calling
    updateStateErr()."""

    def __init__(self, 
                 timestep, 
                 height, 
                 numlookahead, 
                 R=1e-6,
                 Qe=1, 
                 Qdpos=0, 
                 Qdvel=0, 
                 Qdaccel=0,
                 g=9.8, 
                 debug=False):

        """Initialize this ZmpPreview object. Parameters:

           - timestep: delta t, in seconds
           - height: the height of the center of mass, in meters
           - numlookahead: the size of the preview window, in timesteps
           - R: penalty on controls
           - Qe: penalty on ZMP error
           - Qdpos: penalty on differential CoM position
           - Qdvel: penalty on differential CoM velocity
           - Qdaccel: penalty on differential CoM acceleration
           - g: acceleration due to gravity, m/s^2

        See the papers for descriptions of the penalties."""

        self.debug = debug

        T = timestep
        h = height
        nl = numlookahead
        g = g

        A = np.matrix([ [ 1, T, T**2/2 ],
                        [ 0, 1, T ],
                        [ 0, 0, 1 ] ])

        B = np.matrix([T**3/6, T**2/2, T]).reshape((3,1))

        C = np.matrix([1, 0, -h/g]).reshape((1,3))

        AA = np.vstack(( np.hstack((np.eye(1), C*A)),
                         np.hstack((np.zeros((3,1)), A)) ))

        BB = np.vstack( ( C*B, B ) )

        RR = R

        QQ = np.diag( [Qe, Qdpos, Qdvel, Qdaccel] )


        try:
            PP = scipy.linalg.solve_discrete_are(AA, BB, QQ, RR)
        except:
            PP = np.eye(AA.shape[0], dtype=AA.dtype)
            converged = False
            for i in range(1000): # Awful hack to solve DARE by iteration
                AX = AA.T * PP
                AXA = AX * AA
                AXB = AX * BB
                M = (R+BB.T*PP*BB)[0,0]
                PPnew = AXA - AXB*(1.0/M)*AXB.T + QQ
                relerr = np.linalg.norm(PPnew-PP) / np.linalg.norm(PPnew)
                PP = PPnew
                if relerr < 1e-10:
                    print 'DARE solver converged after {} iterations.'.format(i)
                    converged = True
                    break
            if not converged:
                raise Exception('DARE iterative solver failed to converge!')



        SS = 1.0/(RR + BB.T*PP*BB)[0,0]

        KK = SS*BB.T*PP*AA
        Ke = KK[0,0]
        Kx = KK[0,1:4]

        Ac = AA - BB*KK
        XX = -Ac.T * PP * np.matrix([[1,0,0,0]]).T


        G = np.zeros(nl)
        G[0] = -Ke

        for i in range(1,nl):
            G[i] = (SS * BB.T * XX)[0,0]
            XX = Ac.T * XX

        Ks = Ke

        if (self.debug):
            print "AA = \n", AA
            print "BB = \n", BB
            print "QQ = \n", QQ
            print "RR = \n", RR
            print "XX=\n", XX
            print "Ks=",preview.Ks
            print "Kx=",preview.Kx
            print "G(start) =", G[:4]
            print "G(end) = ", G[-4:]
            print "G.sum() = ", G.sum()
            print "G size = ", len(G)

        self.A = A
        self.B = B
        self.C = C
        self.G = G
        self.Ks = Ks
        self.Kx = Kx
        self.nl = nl
        self.T = T
        self.g = g
        self.h = h

    def initStateErr(self, pos=0, vel=0, accel=0, e=0):
        """Initialize a stateErr object which holds position,
        velocity, acceleration, as a vector, and accumulated ZMP error
        as a scalar."""
        X = np.matrix([[pos, vel, accel]]).T
        return (X, e)

    def updateStateErr(self, stateErr, zmpref):

        """Run the controller. The stateErr argument should be a
        (state, error) tuple (as returned by initStateErr() or this
        function). The zmpref argument should be an array of future
        desired ZMP positions. If zmpref is of less length than the
        lookahead window size, the reference trajectory is padded with
        repeats of the last element.

        This function returns three values: the new stateErr after the
        control is executed, the new ZMP position, and the generated
        control."""

        # extract future zmp trajectory
        zmpref = np.array(zmpref).flatten()
        nref = len(zmpref)
        if nref < self.nl:
            npad = self.nl - nref
            zrng = np.hstack(( zmpref[0:], np.ones(npad)*zmpref[-1] ) )
        else:
            zrng = zmpref[0:self.nl]

        # get state
        X, e = stateErr

        # get control
        u = -self.Ks*e - self.Kx*X - np.dot(self.G, zrng)


        # update state & compute ZMP
        Xnew = self.A*X + self.B*u
        zmp = self.C*Xnew
        enew = e + zmp - zmpref[0]

        if self.debug:
            print "zrng.sum() = ", zrng.sum()
            print "u = " , u 
            print "X = ", Xnew.T
            print "zmp = ", zmp
            print "zmpref(0) = ",zmpref[0] 
            print "e =", enew
            print

        # return new state, ZMP, and control
        return (Xnew, enew), zmp, u


if __name__ == "__main__":

    import matplotlib.pyplot as plt

    ##################################################
    # Set up preview controller

    # Freqency in hz of controller
    ctrlFreq = 200

    # Timestep for simulation
    T = 1.0/ctrlFreq

    # Height of CoM = 1.2 meters
    h = 0.5

    # Number of lookahead steps
    nl = int(round(2.5/T))
    nl = 320

    # Create our preview controller
    preview = ZmpPreview(T, h, nl, R=1e-6)

    ##################################################
    # Now set up a simple forward/backward trajectory

    # Compute 10 phases
    numPhases = 10
    phaseTicks = int(round(1.0/T))
    totalTicks = phaseTicks * numPhases

    # Move side/side (Y) by 0.08m each step
    sway = 0.2

    # Move forward (X) by 0.15m each step
    step = 0.15

    # Make a time vector for plotting later
    time = np.arange(totalTicks)*T

    # Now create our zmp reference trajectory for both X & Y
    zref = np.zeros((2,totalTicks))

    for i in range(numPhases):

        if i < 2 or i+2 >= numPhases:
            s = 0
        elif i % 2:
            s = -1
        else:
            s = 1

        if i < 2:
            t = 0
        elif i+2 >= numPhases:
            t = numPhases-3
        else:
            t = i-1

        idx0 = phaseTicks*i
        idx1 = phaseTicks*(i+1)
        zref[0,idx0:idx1] = t*step*np.ones(phaseTicks)
        zref[1,idx0:idx1] = s*sway*np.ones(phaseTicks)

    ##################################################
    # Time to test out our controller

    # Allocate space to store com pos and zmp pos
    coms = np.zeros((2,totalTicks))
    zmps = np.zeros((2,totalTicks))

    # This object is a tuple holding (state, error)
    stateErr = [ preview.initStateErr(),
                 preview.initStateErr() ]

    # Run through our array

    for d in [1]:
        zmp = 0
        for i in range(totalTicks):
            coms[d,i] = stateErr[d][0][0]
            zmps[d,i] = zmp
            stateErr[d], zmp, u = preview.updateStateErr(stateErr[d], zref[d,i:])

    for d in [1]:
        plt.subplot(2,1,d+1)
        plt.plot(time, zref[d,:], label='ZMP Reference')
        plt.plot(time, coms[d,:], label='COM pos')
        plt.plot(time, zmps[d,:], label='ZMP')
        if d == 0:
            plt.legend(loc='upper left', prop={'size':10})
            plt.title('ZMP preview controller - inputs & outputs')
            plt.ylabel('X (meters)')
        else:
            plt.legend(loc='lower left', prop={'size':10})
            plt.ylabel('Y (meters)')
            plt.xlabel('Time (seconds)')

    plt.show()







    
