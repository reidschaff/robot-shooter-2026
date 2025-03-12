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

package org.tahomarobotics.robot.windmill.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants;
import org.tinylog.Logger;

public class WindmillCommands {
    public static Command createCalibrateCommand(Windmill windmill) {
        String FINALIZE_KEY = "Finalize";

        Command cmd = (
            Commands.waitUntil(() -> SmartDashboard.getBoolean(FINALIZE_KEY, false))
                    .beforeStarting(() -> {
                        SmartDashboard.putBoolean(FINALIZE_KEY, false);
                        windmill.disableBrakeMode();
                        Logger.info("Calibrating windmill...");
                    }).finallyDo(interrupted -> {
                        if (interrupted) {
                            Logger.info("Cancelling windmill calibration.");
                            windmill.enableBrakeMode();
                        } else {
                            Logger.info("Windmill calibrated!");
                            windmill.calibrate();
                        }
                    })
        ).onlyWhile(RobotState::isDisabled).ignoringDisable(true);
        cmd.addRequirements(windmill);

        return cmd;
    }

    public static Command createUserButtonCalibrateCommand(Windmill windmill) {
        String FINALIZE_KEY = "User-Finalize";

        return windmill.runOnce(() -> {
            Logger.error("TRIGGERED USER BUTTON");

            if (SmartDashboard.getBoolean(FINALIZE_KEY, false)) {
                Logger.info("Windmill calibrated!");
                windmill.calibrate();
                SmartDashboard.putBoolean(FINALIZE_KEY, false);
            } else {
                windmill.disableBrakeMode();
                Logger.info("Calibrating windmill...");
                SmartDashboard.putBoolean(FINALIZE_KEY, true);
            }
        }).onlyWhile(RobotState::isDisabled).ignoringDisable(true);
    }

    public static Command createElevatorZeroCommand(Windmill windmill) {
        Timer timer = new Timer();

        return new FunctionalCommand(() -> {
            Logger.info("Zeroing elevator with ~movement~.");
            windmill.setElevatorVoltage(WindmillConstants.ELEVATOR_ZEROING_VOLTAGE);
            timer.restart();
        }, () -> {}, interrupted -> {
            windmill.stopElevator();
            if (interrupted) { return; }

            Logger.info("Elevator Zeroed!");
            windmill.calibrate();

            timer.stop();
        }, () -> !windmill.isElevatorMoving() && timer.hasElapsed(WindmillConstants.ELEVATOR_ZEROING_TIMEOUT), windmill)
            .andThen(
                windmill.createResetToPreviousState()
            );
    }
}
