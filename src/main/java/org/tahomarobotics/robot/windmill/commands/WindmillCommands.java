package org.tahomarobotics.robot.windmill.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.windmill.Windmill;
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
}
