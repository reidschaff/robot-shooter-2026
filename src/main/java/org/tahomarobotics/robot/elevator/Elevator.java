package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.elevator.commands.ElevatorZeroCommand;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.SysIdTest;

import static org.tahomarobotics.robot.elevator.ElevatorConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Elevator extends SubsystemIF {

    public static final Logger logger = LoggerFactory.getLogger(Elevator.class);
    private static final Elevator INSTANCE = new Elevator();
    @Logged
    private double targetHeight;
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    TalonFX elevatorRight;
    TalonFX elevatorLeft;

    public SysIdTest sysIdTest;

    //Rotations
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Current> elevatorCurrent;

    public static Elevator getInstance() {
        return INSTANCE;
    }

    private Elevator() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        elevatorRight = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);
        elevatorLeft = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);

        configurator.configureTalonFX(elevatorRight, elevatorConfig, elevatorLeft, false);

        sysIdTest = new SysIdTest(this, elevatorRight);

        motorPosition = elevatorRight.getPosition();
        elevatorVelocity = elevatorRight.getVelocity();
        elevatorCurrent = elevatorRight.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, elevatorCurrent, motorPosition, elevatorVelocity);

        ParentDevice.optimizeBusUtilizationForAll(elevatorRight, elevatorLeft);

    }

    public void zero() {
        elevatorRight.setPosition(0);
    }

    public enum ElevatorStates {
        HIGH,
        MID,
        LOW
    }

    @Logged(name = "height")
    public double getElevatorHeight() {
        return motorPosition.getValueAsDouble();
    }

    public double getElevatorPos(ElevatorStates elevatorState) {
        return switch (elevatorState) {
            case MID -> ELEVATOR_MID_POSE;
            case HIGH -> ELEVATOR_HIGH_POSE;
            default -> ELEVATOR_LOW_POSE;
        };
    }

    public void setElevatorHeight(double height) {
        targetHeight = MathUtil.clamp(height, ELEVATOR_MIN_POSE, ELEVATOR_MAX_POSE);
        elevatorRight.setControl(positionControl.withPosition(targetHeight));
    }

    @Logged
    public boolean isAtPosition() {
        return Math.abs(targetHeight - getElevatorHeight()) <= ElevatorConstants.POSITION_TOLERANCE;
    }

    @Logged
    public boolean isMoving() {
        return Math.abs(elevatorVelocity.refresh().getValueAsDouble()) > VELOCITY_TOLERANCE;
    }

    public void setVelocity(double speed) {
        setElevatorHeight(targetHeight + speed * Robot.kDefaultPeriod);
    }

    public void setVoltage(double voltage) { elevatorRight.setVoltage(voltage); }

    public void stop() {
        elevatorRight.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
        SmartDashboard.putNumber("Target Position:", targetHeight);
        SmartDashboard.putNumber("Motor Voltage", elevatorRight.getMotorVoltage().getValueAsDouble());
//        SmartDashboard.putNumber("REAL Target Position", elevatorRight.getAppliedControl() instanceof ControlRequest ? );
        BaseStatusSignal.refreshAll(motorPosition, elevatorVelocity, elevatorCurrent);
    }

    @Override
    public SubsystemIF initialize() {
        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ElevatorZeroCommand())
                .ignoringDisable(true).schedule();

        return this;
    }
}