package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;

import java.util.function.Consumer;

public class PathPlannerHelper {
    public static SendableChooser<Command> getAutoChooser(Chassis chassis, Consumer<Command> onChange) {
        AutoBuilder.configure(
            chassis::getPose,
            chassis::resetOdometry,
            chassis::getChassisSpeeds,
            chassis::autoDrive,
            new PPHolonomicDriveController(
                ChassisConstants.AUTO_TRANSLATION_PID,
                ChassisConstants.AUTO_ROTATION_PID,
                0.02
            ),
            ChassisConstants.robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.map(value -> value.equals(DriverStation.Alliance.Red)).orElse(false);
            },
            chassis
        );

        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();

        Command defaultCommand = Commands.none().withName(AutonomousConstants.DEFAULT_AUTO_NAME);

        chooser.setDefaultOption(AutonomousConstants.DEFAULT_AUTO_NAME, defaultCommand);

        chooser.onChange(onChange);

        return chooser;
    }

}