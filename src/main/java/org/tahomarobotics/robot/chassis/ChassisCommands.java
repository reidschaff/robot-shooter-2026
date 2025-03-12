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

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.windmill.Windmill;

import java.util.function.DoubleSupplier;

public class ChassisCommands {
    public static Command createAlignSwerveCommand(Chassis chassis) {
        String FINALIZE_KEY = "Finalize";

        Command cmd = (
            Commands.waitUntil(() -> SmartDashboard.getBoolean(FINALIZE_KEY, false))
                    .beforeStarting(() -> {
                        SmartDashboard.putBoolean(FINALIZE_KEY, false);
                        chassis.initializeCalibration();
                    })
                    .finallyDo(interrupted -> {
                        if (interrupted) {
                            chassis.cancelCalibration();
                        } else {
                            chassis.finalizeCalibration();
                        }
                    })
        ).onlyWhile(RobotState::isDisabled).ignoringDisable(true);
        cmd.addRequirements(chassis);

        return cmd;
    }

    public static Command createTeleOpDriveCommand(
        Chassis chassis,
        DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega
    ) {
        double maxAngularVelocity = ChassisConstants.MAX_ANGULAR_VELOCITY;

        return chassis.runEnd(
            () -> {
                double maxVelocity = Windmill.getInstance().isScoringCoral() ? ChassisConstants.SLOW_VELOCITY : ChassisConstants.MAX_VELOCITY;

                double direction = DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red ? -1.0 : 1.0;

                chassis.drive(new ChassisSpeeds(
                    x.getAsDouble() * maxVelocity * direction,
                    y.getAsDouble() * maxVelocity * direction,
                    omega.getAsDouble() * maxAngularVelocity
                ));
            },
            () -> chassis.drive(new ChassisSpeeds())
        );
    }
}
