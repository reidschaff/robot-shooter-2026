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

package org.tahomarobotics.robot.util.sysid;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.tahomarobotics.robot.util.SubsystemIF;

/**
 * A helper class for characterizing motors with SysId using Phoenix 6's {@link SignalLogger}.
 */
public class SysIdTests {
    // -- Characterization --

    public static Test characterize(
        String testName,
        SubsystemIF subsystem,
        TalonFX motor,
        Velocity<VoltageUnit> rampRate,
        Voltage stepSize
    ) {
        return characterize(testName, subsystem, motor, rampRate, stepSize, null, false);
    }

    public static Test characterize(
        String testName,
        SubsystemIF subsystem,
        TalonFX motor,
        Velocity<VoltageUnit> rampRate,
        Voltage stepSize,
        TalonFX follower,
        boolean inverted
    ) {
        if (follower != null) {
            follower.setControl(new Follower(motor.getDeviceID(), inverted));
        }

        // Ensure the signals are updating

        BaseStatusSignal.setUpdateFrequencyForAll(
            250,
            motor.getPosition(),
            motor.getVelocity(),
            motor.getMotorVoltage()
        );

        // Create SysId routine

        VoltageOut voltageControl = new VoltageOut(0);

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                rampRate,
                stepSize,
                null,
                state -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> motor.setControl(voltageControl.withOutput(volts)),
                null,
                subsystem
            )
        );

        return new Test(
            testName,
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            routine.quasistatic(SysIdRoutine.Direction.kReverse),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            routine.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    // -- Record(s) --

    public record Test(String name,
                       Command quasistaticForward, Command quasistaticReverse,
                       Command dynamicForward, Command dynamicReverse) {}
}
