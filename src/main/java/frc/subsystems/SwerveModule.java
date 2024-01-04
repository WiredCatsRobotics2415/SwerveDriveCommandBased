package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.RobotMap.Swerve;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Constants.Swerve.SwerveModuleName;
import frc.utils.AbsoluteAnalogEncoder;
import frc.utils.PIDValue;
import frc.utils.RobotPreferences;

public class SwerveModule implements Sendable {
    //HARDWARE
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX azimuthMotor;
    private AbsoluteAnalogEncoder absoluteEncoder;
    
    //PROPERTIES
    public SwerveModuleName name;
    private boolean autoAzimuthSync;

    //STATES
    private SwerveModuleState lastState = new SwerveModuleState();
    private int encoderSyncTick = 0;

    public SwerveModule(SwerveModuleName name) {
        this.name = name;
        this.driveMotor = new WPI_TalonFX(Swerve.DRIVE_PORTS[name.ordinal()]);
        this.azimuthMotor = new WPI_TalonFX(Swerve.AZIMUTH_PORTS[name.ordinal()]);
        this.absoluteEncoder = new AbsoluteAnalogEncoder(
            Swerve.ABSOLUTE_ENCODER_PORTS[name.ordinal()],
            RobotPreferences.getOffsetOfModule(name),
            Swerve.ABSOLUTE_ENCODER_REV[name.ordinal()]);
        
        configDriveMotor();
        configAzimuthMotor(Constants.Swerve.AZIMUTH_PIDS[name.ordinal()]);

        autoAzimuthSync = RobotPreferences.getAutoAzimuthSync();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault(Constants.CAN_TIMEOUT_MS);
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT_MS);
        driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Constants.CAN_TIMEOUT_MS);
        // Sets the drive talon to use the built-in motor sensor for PID
        driveMotor.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT_MS);
        // Zero the internal driving encoder (assume velocity 0 on startup)
        driveMotor.setNeutralMode(NeutralMode.Brake); 
        // The wheel shouldn't rotate when not being driven.
        driveMotor.setInverted(RobotMap.Swerve.DRIVE_REVERSED[name.ordinal()]); 
        // Is the motor reversed relative to others?
        driveMotor.configSupplyCurrentLimit(Constants.Swerve.DRIVE_CURRENT_LIMIT);
        // current limits (these are kinda arbitrary)
        driveMotor.configOpenloopRamp(0.0, Constants.CAN_TIMEOUT_MS);
        driveMotor.configClosedloopRamp(0.0, Constants.CAN_TIMEOUT_MS);
    }

    private void configAzimuthMotor(PIDValue pid) {
        azimuthMotor.configFactoryDefault(Constants.CAN_TIMEOUT_MS);
        //NOTE: uncomment the following only if using interal encoders for PID loop
        azimuthMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT_MS);
        // Use the internal TalonFX encoder for PID
        azimuthMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
                Constants.CAN_TIMEOUT_MS);
        // The internal encoder should startup at zero
        azimuthMotor.setSelectedSensorPosition(Constants.Swerve.degreesToFalcon(this.absoluteEncoder.getRotationDegrees()), 0,
                Constants.CAN_TIMEOUT_MS);
        // Set the internal encoder equal to the converted value from the external
        // encoder (make sure it's accurate)
        azimuthMotor.setNeutralMode(NeutralMode.Coast);
        // Allows some passive rotation correction (like shopping cart wheels)
        azimuthMotor.setInverted(RobotMap.Swerve.AZIMUTH_REVERSED[name.ordinal()]);
        // Is the motor reversed relative to others?
        azimuthMotor.configSupplyCurrentLimit(Constants.Swerve.AZIMUTH_CURRENT_LIMIT);
        // PID Values (for position, use P and D terms to ensure accuracy)
        //NOTE: only uncomment below if using internal encoder for PID
        azimuthMotor.config_kP(0, pid.getKP(), Constants.CAN_TIMEOUT_MS);
        azimuthMotor.config_kI(0, pid.getKI(), Constants.CAN_TIMEOUT_MS);
        azimuthMotor.config_kD(0, pid.getKD(), Constants.CAN_TIMEOUT_MS);
    }

    /**
    * Returns the current state of the module (m/s and angle).
    *
    * @return The current state of the module.
    */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Constants.Swerve.falconToMPS(driveMotor.getSelectedSensorVelocity()),
            Rotation2d.fromDegrees(absoluteEncoder.getRotationDegrees())
        );
    }

    /**
    * Returns the current position of the module (drive distance and angle).
    *
    * @return The current position of the module.
    */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Constants.Swerve.driveFalconToDistance(driveMotor.getSelectedSensorPosition()),
            Rotation2d.fromDegrees(absoluteEncoder.getRotationDegrees())
        );
    }

    /**
     * Falcon encoders report values over 360, so this function takes an angle [0-360]
     * and the current value of the motor, and returns a value that is whatever angle was given
     * but in the 0-360 range of wherever the motor is.
     * For example: an 88 degree angle on a motor that is at 721 degrees would be 809
     */
    private double over360RangeSetter(double desiredSetAngle, double currentAngle) {
        if (Math.floor(currentAngle / 360.0) != 0) {
            // if the current angle is past the 0-360 range, desired set angle must be
            // adjusted to the current range of the motor angle
            // always positive modulus function = (a % b + b) % b - positive modulus
            // function faster than conditionals
            // taking the angle in [0, 360) then adding the number of full rotations * 360
            desiredSetAngle = ((desiredSetAngle % 360 + 360) % 360)
                    + Math.floor(currentAngle / 360.0) * 360.0;
        }
        return desiredSetAngle;
    }

    /**
   * Sets the new state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setNewState(SwerveModuleState desiredState) {
    //Auto encoder sync
    //Borrowed from https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500SteerControllerFactoryBuilder.java
    if (autoAzimuthSync && azimuthMotor.getSelectedSensorVelocity() < Constants.Swerve.ENCODER_RESET_CPR_THRESHOLD) {
        if (++encoderSyncTick >= Constants.Swerve.ENCODER_RESET_TICKS) {
            encoderSyncTick = 0;
            double currentAngle = Constants.Swerve.falconToDegrees(azimuthMotor.getSelectedSensorPosition());
            azimuthMotor.setSelectedSensorPosition(Constants.Swerve.degreesToFalcon(
                over360RangeSetter(absoluteEncoder.getRotationDegrees(), currentAngle)
            ), 0, Constants.CAN_TIMEOUT_MS);
            System.out.println("[MODULE " + name.toString() + "] Azimuth synced, diff: " + Math.abs((currentAngle%360)-absoluteEncoder.getRotationDegrees()));
        }
    } else {
        encoderSyncTick = 0;
    }
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(absoluteEncoder.getRotationDegrees()));
    //SwerveModuleState state = desiredState;
    
    azimuthMotor.set(ControlMode.Position, Constants.Swerve.degreesToFalcon(state.angle.getDegrees()));
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.Swerve.MAX_DRIVE_SPEED);
    
    //Simulation - fill in all of the simulated values with the ideal 
    if (Robot.isSimulation()) {
        azimuthMotor.getSimCollection().setIntegratedSensorRawPosition((int)Constants.Swerve.degreesToFalcon(state.angle.getDegrees()));
        driveMotor.getSimCollection().addIntegratedSensorPosition((int)((state.speedMetersPerSecond/50)*(Constants.Swerve.DRIVE_GEAR_RATIO/Constants.Swerve.WHEEL_CIRCUMFERENCE)*2048));
        driveMotor.getSimCollection().setIntegratedSensorVelocity((int)Constants.Swerve.MPSToFalcon(state.speedMetersPerSecond));
    }
    lastState = state;
  }

  public SwerveModuleState getLastSetState() {
      return lastState;
  }

  public void setModuleOffset(double offset) {
    this.absoluteEncoder.setOffset(offset);
  }

  @Override public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModuleState");
    builder.addDoubleProperty("setSpeed", () -> this.getLastSetState().speedMetersPerSecond, null);
    builder.addDoubleProperty("setAngle", () -> this.getLastSetState().angle.getDegrees(), null);
    builder.addDoubleProperty("actualSpeed", () -> this.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty("actualAngle", () -> this.getState().angle.getDegrees(), null);
    builder.addDoubleProperty("externalAngle", () -> {
       return Constants.Swerve.falconToDegrees(azimuthMotor.getSelectedSensorPosition()-1)%360;
    }, null);
  }
}
