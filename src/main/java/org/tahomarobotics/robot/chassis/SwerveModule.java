package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;

import java.util.List;

import static org.tahomarobotics.robot.chassis.ChassisConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class SwerveModule {
    private static final Logger logger = LoggerFactory.getLogger(SwerveModule.class);

    private final String name;
    private final Translation2d translationOffset;
    @Logged
    private double angularOffset;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    @Logged
    private SwerveModuleState targetState = new SwerveModuleState();

    private final StatusSignal<Angle> steerPosition;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<AngularAcceleration> driveAcceleration;
    private final StatusSignal<Current> driveCurrent, steerCurrent;

    private final VelocityVoltage driveMotorVelocity = new VelocityVoltage(0.0).withEnableFOC(RobotConfiguration.CANIVORE_PHOENIX_PRO);
    private final PositionDutyCycle steerMotorPosition = new PositionDutyCycle(0.0).withEnableFOC(RobotConfiguration.CANIVORE_PHOENIX_PRO);

    private final RobustConfigurator configurator;

    public SwerveModule(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        configurator = new RobustConfigurator(logger);

        name = descriptor.moduleName();
        translationOffset = descriptor.offset();
        this.angularOffset = angularOffset;

        driveMotor = new TalonFX(descriptor.driveId(), RobotConfiguration.CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerId(), RobotConfiguration.CANBUS_NAME);
        steerEncoder = new CANcoder(descriptor.encoderId(), RobotConfiguration.CANBUS_NAME);

        configurator.configureTalonFX(driveMotor, driveMotorConfiguration, descriptor.moduleName() + " drive motor");
        configurator.configureTalonFX(steerMotor, steerMotorConfiguration, descriptor.encoderId(), descriptor.moduleName() + " steer motor");
        configurator.configureCancoder(steerEncoder, encoderConfiguration, angularOffset, descriptor.moduleName() + " encoder");

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();

        steerPosition = steerEncoder.getPosition();
        steerVelocity = steerEncoder.getVelocity();

        driveCurrent = driveMotor.getSupplyCurrent();
        steerCurrent = steerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY,
                drivePosition,
                driveVelocity,
                driveAcceleration,
                steerVelocity,
                driveCurrent,
                steerCurrent
        );
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, steerEncoder);
    }

    // CALIBRATION

    public void initCalibration() {
        configurator.setCancoderAngularOffset(steerEncoder, 0);
        configurator.setMotorNeutralMode(steerMotor, NeutralModeValue.Coast);
    }

    public double finalizeCalibration() {
        angularOffset = -steerPosition.refresh().getValueAsDouble();
        configurator.setCancoderAngularOffset(steerEncoder, angularOffset);
        configurator.setMotorNeutralMode(steerMotor, NeutralModeValue.Brake);
        return angularOffset;
    }

    public void cancelCalibration() {
        configurator.setCancoderAngularOffset(steerEncoder, angularOffset);
    }

    // GETTERS

    public Translation2d getTranslationOffset() {
        return translationOffset;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRotations(getSteerAngle()));
    }

    @Logged(name = "steerAngle")
    public double getSteerAngle() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPosition, steerVelocity);
    }

    @Logged(name = "drivePosition")
    public double getDrivePosition() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(drivePosition, driveVelocity) * DRIVE_POSITION_COEFFICIENT;
    }

    @Logged(name = "driveVelocity")
    public double getDriveVelocity() {
        return driveVelocity.getValueAsDouble() * DRIVE_POSITION_COEFFICIENT;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRotations(getSteerAngle()));
    }

    @Logged(name = "driveCurrent")
    public double getDriveCurrent() {
        return driveCurrent.getValueAsDouble();
    }

    @Logged(name = "steerCurrent")
    public double getSteerCurrent() {
        return steerCurrent.getValueAsDouble();
    }

    public void setDesiredState(SwerveModuleState state) {
        targetState = state;
    }

    public List<BaseStatusSignal> getStatusSignals() {
        return List.of(
                drivePosition,
                driveAcceleration,
                driveVelocity,
                steerPosition,
                steerVelocity,
                driveCurrent,
                steerCurrent
        );
    }

    public void periodic() {
        updateDesiredState();
    }

    public void updateDesiredState() {
        double steerAngle = getSteerAngle();

        targetState.optimize(Rotation2d.fromRotations(steerAngle));
        targetState.angle = Rotation2d.fromRotations((targetState.angle.getRotations() % 1.0 + 1.0) % 1.0);
        targetState.speedMetersPerSecond *= targetState.angle.minus(Rotation2d.fromRotations(steerAngle)).getCos();

        driveMotor.setControl(driveMotorVelocity.withVelocity(targetState.speedMetersPerSecond / DRIVE_POSITION_COEFFICIENT));
        steerMotor.setControl(steerMotorPosition.withPosition(targetState.angle.getRotations()));
    }
}
