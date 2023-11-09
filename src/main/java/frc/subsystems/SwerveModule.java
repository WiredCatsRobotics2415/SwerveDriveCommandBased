package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
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

    //STATES
    private SwerveModuleState lastState = new SwerveModuleState();
    private int encoderSyncTick = 0;

    //CONTROLLERS
    private SimpleMotorFeedforward azimuthMotorFF;
    private ProfiledPIDController azimuthPID;
    
    private SimpleMotorFeedforward driveMotorFF;
    private PIDController driveMotorPID; 

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

        azimuthMotorFF = Constants.Swerve.AZIMUTH_FFS[name.ordinal()];
        azimuthPID = new ProfiledPIDController(Constants.Swerve.AZIMUTH_PIDS[name.ordinal()].getKP(),
        0,
        Constants.Swerve.AZIMUTH_PIDS[name.ordinal()].getKD(),
            new TrapezoidProfile.Constraints(
                Constants.Swerve.MAX_ANGULAR_SPEED,
                2*Math.PI //Note: Copied from WPI example
            )
        );
        azimuthPID.enableContinuousInput(-Math.PI, Math.PI);

        driveMotorFF = Constants.Swerve.DRIVE_FF;
        driveMotorPID = new PIDController(
            Constants.Swerve.DRIVE_PID.getKP(),
            Constants.Swerve.DRIVE_PID.getKI(), 
            Constants.Swerve.DRIVE_PID.getKD()
        );
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
   * Sets the new state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setNewState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(absoluteEncoder.getRotationDegrees()));
    //SwerveModuleState state = desiredState;
    
    //#region Azimuth WPI PID
    final double turnOutput = azimuthPID.calculate(Math.toRadians(absoluteEncoder.getRotationDegrees()), state.angle.getRadians());
    final double turnFeedforward = azimuthMotorFF.calculate(azimuthPID.getSetpoint().velocity);
    azimuthMotor.setVoltage(turnOutput + turnFeedforward);
    //#endregion
    
    //#region Azimuth TalonFX loop
    azimuthMotor.set(ControlMode.Position, Constants.Swerve.degreesToFalcon(state.angle.getDegrees()));
    //#endregion

    //#region Drive WPI PID
    final double driveOutput = driveMotorPID.calculate(Constants.Swerve.falconToMPS(driveMotor.getSelectedSensorVelocity()), state.speedMetersPerSecond);
    final double driveFeedForward = driveMotorFF.calculate(state.speedMetersPerSecond);

    driveMotor.setVoltage(driveOutput + driveFeedForward);
    //#endregion

    //#region Drive % Out    
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.Swerve.MAX_DRIVE_SPEED);
    //#endregion
    
    if (Robot.isSimulation()) {
        azimuthMotor.getSimCollection().setIntegratedSensorRawPosition((int)Constants.Swerve.degreesToFalcon(state.angle.getDegrees()));
        driveMotor.getSimCollection().addIntegratedSensorPosition((int)(driveMotor.getSelectedSensorPosition()*Constants.Swerve.DRIVE_DISTANCE_PER_ROTATION));
        driveMotor.getSimCollection().setIntegratedSensorVelocity((int)Constants.Swerve.MPSToFalcon(state.speedMetersPerSecond));
    }

    lastState = state;
  }

  public SwerveModuleState getLastSetState() {
      return lastState;
  }

  public void setModuleOffsetFromStorage() {
    this.absoluteEncoder.setOffset(RobotPreferences.getOffsetOfModule(name));
  }

  @Override public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModuleState");
    builder.addDoubleProperty("setSpeed", () -> this.getLastSetState().speedMetersPerSecond, null);
    builder.addDoubleProperty("setAngle", () -> this.getLastSetState().angle.getDegrees(), null);
    builder.addDoubleProperty("actualSpeed", () -> this.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty("actualAngle", () -> this.getState().angle.getDegrees(), null);
    //builder.addDoubleProperty("externalAngle", () -> Constants.Swerve.falconToDegrees(azimuthMotor.getSelectedSensorPosition()), null);
  }
}
