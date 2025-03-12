/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.tahomarobotics.robot.windmill.Windmill;

public class SwerveDriveLimiter {


    private SwerveModuleState[] prevStates;
    private double prevTime;

    public SwerveDriveLimiter(SwerveModuleState[] states) {
        prevStates = states;
        prevTime = MathSharedStore.getTimestamp();
    }

    public SwerveModuleState[] calculate(SwerveModuleState[] states) {
        double accelerationLimit = (Windmill.getInstance().isScoringCoral() && !Chassis.getInstance().isAutoAligning()) ?
            ChassisConstants.SLOW_ACCELERATION_LIMIT : ChassisConstants.ACCELERATION_LIMIT;

        double[] acceleration = new double[states.length];
        SwerveModuleState[] limited = new SwerveModuleState[4];
        System.arraycopy(states, 0, limited, 0, states.length);

        double currentTime = MathSharedStore.getTimestamp();
        double dT = currentTime - prevTime;
        prevTime = currentTime;

        double averageAcceleration = 0;
        boolean brakeMode = true;
        for (int i = 0; i < states.length; i++) {
            brakeMode &= states[i].speedMetersPerSecond == 0.0;
            acceleration[i] = (states[i].speedMetersPerSecond - prevStates[i].speedMetersPerSecond) / dT;
            averageAcceleration += Math.abs(acceleration[i]);
        }
        averageAcceleration /= states.length;

        double scale = averageAcceleration > accelerationLimit ? accelerationLimit / averageAcceleration : 1.0;

        for (int i = 0; i < states.length; i++) {
            limited[i].speedMetersPerSecond = brakeMode ? 0.0 : prevStates[i].speedMetersPerSecond + scale * acceleration[i] * dT;
        }

        prevStates = limited;
        return limited;
    }
}
