#
# Created by Jose Rojas, 9/24/17
#

import numpy as np
import rospy
import matplotlib.pyplot as plt
import time

def permute_parameter(center, offset, inc, useHigh, useLow, func):
    """
     * Permutation helper function. Does a uniform optimized search from a center point, outward across a range of values.
     * The lambda function serves as the implementation for the search.
     * Returning true from the lambda will break the loop early.
    """

    p = 0
    factor = 0
    offset = abs(offset)
    diff = int(useHigh) + int(useLow) * int(offset / inc)

    while factor <= diff:

        if useHigh and useLow:
            sign = -1.0 if factor % 2 == 0 else 1.0
        elif useHigh:
            sign = 1.0
        else:
            sign = -1.0 if useLow else 0.0

        mult = ((factor + 1) / 2) if useHigh and useLow else factor

        p = center + mult * sign * inc
        if p > 0 and func(p):
            return True

        factor += 1

    return False

class JMT:

    def __init__(self, start, end, t):
        self.params = []
        self.__tmax = t
        self.cost = 0

        """
        Calculate the Jerk Minimizing Trajectory that connects the initial state
        to the final state in time T.

        INPUTS

        start - the vehicles start location given as a length three array
            corresponding to initial values of [s, s_dot, s_double_dot]

        end   - the desired end state for vehicle. Like "start" this is a
            length three array.

        T     - The duration, in seconds, over which this maneuver should occur.

        OUTPUT
        an array of length 6, each value corresponding to a coefficent in the polynomial
        s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

        EXAMPLE

        > jmt( [0, 10, 0], [10, 10, 0], 1)
        [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
        """

        t2 = t**2
        t3 = t**3
        t4 = t**4
        t5 = t**5

        A = np.array([[t3, t4, t5],
                      [3 * t2, 4 * t3, 5 * t4],
                      [6 * t, 12 * t2, 20 * t3]])

        b = np.array([end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
                      end[1] - (start[1] + start[2] * t),
                      end[2] - start[2]])

        x = np.linalg.solve(A, b)
        self.params = [start[0], start[1], 0.5 * start[2], x[0], x[1], x[2]]

        #rospy.loginfo("   jmt params {}".format(self.params))

    def linearize(self, accel):
        self.params = [self.params[0], self.params[1], 0.5 * accel, 0, 0, 0]

    def is_defined(self):
        return not(self.params.empty())

    def position(self, t):
        t1 = min(t, self.__tmax)
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        val = self.params[0] + self.params[1] * t1 + self.params[2] * t2 + \
              self.params[3] * t3 + self.params[4] * t4 + self.params[5] * t5
        return val + (self.speed(t) * ((t - self.__tmax) if t > self.__tmax else 0))

    def speed(self, t):
        t = min(t, self.__tmax)
        t2 = t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        return self.params[1] + 2 * self.params[2] * t2 + 3 * self.params[3] * t3 + \
               4 * self.params[4] * t4 + 5 * self.params[5] * t5

    def accel(self, t):
        t3 = t
        t4 = t3 * t
        t5 = t4 * t
        return 0 if t > self.__tmax else 2 * self.params[2] + 6 * self.params[3] * t3 + \
                                         12 * self.params[4] * t4 + 20 * self.params[5] * t5

    def jerk(self, t):
        t1 = min(t, self.__tmax)
        t4 = t1
        t5 = t4 * t1
        return 0 if t > self.__tmax else 6 * self.params[3] + 24 * self.params[4] * t4 + 60 * self.params[5] * t5

    def time_for_position(self, s):
        # reverse parameters roots requires highest polynomial term to be first
        terms = self.params[::-1]
        terms[-1] -= s
        roots = np.roots(terms)
        roots = roots[np.isreal(roots)]

        #rospy.loginfo("   root terms {}, roots {}".format(terms, roots))

        closest = np.min(roots) if len(roots) else self.__tmax

        return closest if closest >= 0 else np.max(roots)  # time cannot be negative

    def t(self):
        return self.__tmax

    @staticmethod
    def validate(jmt, limits, dT, dt):

        # does it violate acceleration
        failedConstraint = False

        t = 0
        while t < dT and not failedConstraint:
            accel = jmt.accel(t)
            if abs(accel) > limits[1]:
                failedConstraint = True
                rospy.loginfo("failed accel constraint")
                break
            t += dt

        # does it violate speed
        t = 0
        while t < dT and not failedConstraint:
            speed = jmt.speed(t)
            if abs(speed) > limits[0]:
                failedConstraint = True
                rospy.loginfo("failed speed constraint, {} vs {}".format(speed, limits[0]))
                break
            t += dt

        # does it violate jerk
        t = 0
        while t < dT and not failedConstraint:
            jerk = jmt.jerk(t)
            if abs(jerk) > limits[2]:
                rospy.loginfo("failed jerk constraint")
                failedConstraint = True
                break
            t += dt

        return not failedConstraint

    @staticmethod
    def cost_function(target, estimate):
        # we mostly care about the speed
        SPEED_WEIGHT = 5.0
        posDiff = target[0] - estimate[0]
        speedDiff = target[1] - estimate[1]
        accelDiff = target[2] - estimate[2]
        return speedDiff * speedDiff * SPEED_WEIGHT + posDiff * posDiff + accelDiff * accelDiff

    @staticmethod
    def search_jmts(start, meanEnd, stdDeviation, limits, numberOfStateSamples, dT, dt, dvT):

        #permute states
        map = {}

        def func(time_interval):
            pdf = np.array([
                np.random.uniform(meanEnd[0] - stdDeviation[0], meanEnd[0], size=numberOfStateSamples),
                np.random.normal(meanEnd[1], stdDeviation[1], size=numberOfStateSamples),
                np.random.normal(meanEnd[2], stdDeviation[2], size=numberOfStateSamples)
            ])

            # rospy.loginfo("search: pdf {}".format(pdf))
            rospy.loginfo("PDF for time interal: {} ".format(time_interval))


            if time_interval > 0:

                for i in range(0, numberOfStateSamples):
                    end = pdf[:,i]

                    rospy.loginfo("search: start {}, end {}".format(start, end))


                    jmt = JMT(start, end, time_interval)

                    rospy.loginfo("search: start {}, end {}, found : {}".format(start, end, jmt.params))

                    if end[1] >= 0 and JMT.validate(jmt, limits, time_interval, dt):
                        jmt.cost = JMT.cost_function(meanEnd, end)
                        # if JMTs share the lowest cost, they will be overwritten, but that's generally okay for this purpose
                        map[jmt.cost] = jmt

            return len(map) != 0

        permute_parameter(dT, dvT, dt, True, True, func)

        # print("Found {} jmts".format(len(map)))
        rospy.loginfo("search: jmts found {}".format(len(map)))

        return sorted(map.values(), key=map.get)

def test():
    start = [10,5,0]
    meanend = [50,0,0]
    meansd = [1,0,2]
    limits = [20, 10, 10]
    print 'searching JMTs'
    t = 10
    s = time.time()
    j = JMT.search_jmts(start,meanend,meansd, limits, 20, t, 1, 4)
    print(time.time() - s)
    figt = np.arange(0,t,0.1)
    plt.figure(1)
    pi = 1
    for i in j:
        print i.params
        if pi > 5:
            break
        i.params.reverse()
        plt.subplot(5, 1, pi)
        plt.plot(figt,np.polyval(i.params, figt),)
        pi += 1
    plt.show()

if __name__ == "__main__":
    test()