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

package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.tahomarobotics.robot.RobotMap.*;

public class ShooterCommands {
    static PositionVoltage posControl = new PositionVoltage(0);
    static VelocityVoltage velControl = new VelocityVoltage(0);
    static TalonFX Pivotmotor = new TalonFX(PIVOT_MOTOR);
    static TalonFX FlywheelMotor = new TalonFX(FLYWHEEL_MOTOR);
    static TalonFX Passthroughmotor = new TalonFX(PASSTHROUGH_MOTOR);

    public void Collecting() {
        Pivotmotor.setControl(posControl.withPosition(Degrees.of(45)));
        FlywheelMotor.setControl(velControl.withVelocity(RotationsPerSecond.of(ShooterConstants.FLYWHEEL_SPEED)));
        Passthroughmotor.setControl(velControl.withVelocity(RotationsPerSecond.of(-50)));
        Logger.recordOutput("Pivot.angle", 45);
        Logger.recordOutput("FlywheelMotor.SPEED", 300);
        Logger.recordOutput("PassthroughMotor.SPEED", -50);
    }

    public void FIRE() {
        Passthroughmotor.setControl(velControl.withVelocity(RotationsPerSecond.of(50)));
        FlywheelMotor.setControl(velControl.withVelocity(RotationsPerSecond.of(-ShooterConstants.FLYWHEEL_SPEED)));
        Logger.recordOutput("PassthroughMotor.SPEED", 50);
        Logger.recordOutput("FlywheelMotor.SPEED", -300);
    }

    public void angle90() {
        Pivotmotor.setControl(posControl.withPosition(Degrees.of(90)));
        FlywheelMotor.setControl(velControl.withVelocity(RotationsPerSecond.of(-ShooterConstants.FLYWHEEL_SPEED)));
        Logger.recordOutput("Pivot.angle", 90);
        Logger.recordOutput("FlywheelMotor.SPEED", -300);
    }

    public void angle135() {
        Pivotmotor.setControl(posControl.withPosition(Degrees.of(135)));
        FlywheelMotor.setControl(velControl.withVelocity(RotationsPerSecond.of(-ShooterConstants.FLYWHEEL_SPEED)));
        Logger.recordOutput("Pivot.angle", 135);
        Logger.recordOutput("FlywheelMotor.SPEED", -300);
    }

    public void angle115() {
        Pivotmotor.setControl(posControl.withPosition(Degrees.of(115)));
        FlywheelMotor.setControl(velControl.withVelocity(RotationsPerSecond.of(-ShooterConstants.FLYWHEEL_SPEED)));
        Logger.recordOutput("Pivot.angle", 115);
        Logger.recordOutput("FlywheelMotor.SPEED", -300);
    }
}