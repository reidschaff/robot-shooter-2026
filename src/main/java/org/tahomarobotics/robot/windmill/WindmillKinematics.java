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

package org.tahomarobotics.robot.windmill;

import edu.wpi.first.math.geometry.Translation2d;

import static org.tahomarobotics.robot.windmill.WindmillConstants.*;

public class WindmillKinematics {

    public static class KinematicsException extends Exception {
        public KinematicsException(String message) {
            super(message);
        }
    }

    public static WindmillState inverseKinematics(double time, Translation2d endpoint, WindmillState previousState) throws KinematicsException {
        return inverseKinematics(time, endpoint, previousState, true);
    }

    public static WindmillState inverseKinematics(
        double time, Translation2d endpoint, WindmillState previousState, boolean swingArmUp
    ) throws KinematicsException {
        double endpointXMeters = endpoint.getX();
        double endpointYMeters = endpoint.getY();

        // TODO: Extract endpoint bounds check

        double elevatorHeightMeters;
        double armAngleRadians; // Zero being horizontal to the front of the robot

        if (Double.isNaN(endpointXMeters) || Double.isNaN(endpointYMeters)) {
            throw new KinematicsException("Cannot compute state with a NaN endpoint component! Endpoint: " + endpoint + ".");
        }

        if (endpointYMeters < END_EFFECTOR_MIN_HEIGHT || endpointYMeters > END_EFFECTOR_MAX_HEIGHT) {
            throw new KinematicsException("Endpoint y component is out of bounds: " + endpointYMeters + " meters.");
        }

        armAngleRadians = Math.acos(endpointXMeters / ARM_LENGTH);
        if (!swingArmUp) { armAngleRadians *= -1; }
        elevatorHeightMeters = endpointYMeters - Math.sin(armAngleRadians) * ARM_LENGTH;

        // TODO: The minimum endpoint pose does not work due to collisions with various mechanisms
        if (elevatorHeightMeters < ELEVATOR_MIN_POSE || elevatorHeightMeters > ELEVATOR_MAX_POSE) {
            throw new KinematicsException("Calculated elevator height isn't achievable! Attempting to go to " + elevatorHeightMeters + " meters.");
        }

        return WindmillState.fromPrevious(time, elevatorHeightMeters, armAngleRadians, previousState);
    }

    public static Translation2d forwardKinematics(WindmillState state) {
        return forwardKinematics(state, true);
    }

    public static Translation2d forwardKinematics(WindmillState state, boolean swingArmUp) {
        double elevatorHeightMeters = state.elevatorState().heightMeters();
        double armAngleRadians = state.armState().angleRadians();

        return forwardKinematics(elevatorHeightMeters, armAngleRadians, swingArmUp);
    }

    public static Translation2d forwardKinematics(double elevatorHeightMeters, double armAngleRadians) {
        return forwardKinematics(elevatorHeightMeters, armAngleRadians, true);
    }

    public static Translation2d forwardKinematics(double elevatorHeightMeters, double armAngleRadians, boolean swingArmUp) {
        double endpointXMeters = ARM_LENGTH * Math.cos(armAngleRadians);
        double armYOffsetMeters = ARM_LENGTH * Math.sin(armAngleRadians);
        double endpointYMeters = swingArmUp ? elevatorHeightMeters + armYOffsetMeters : elevatorHeightMeters - armYOffsetMeters;

        return new Translation2d(endpointXMeters, endpointYMeters);
    }
}
