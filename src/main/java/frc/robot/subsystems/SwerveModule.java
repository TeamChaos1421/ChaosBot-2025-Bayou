package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

class TurningEncoder {
  private final CANcoder m_cancoder;
  public TurningEncoder (int ID, double MagnetOffset) {
    m_cancoder = new CANcoder(ID);
  }
  public double getPositionRadians() {
    return m_cancoder.getAbsolutePosition().getValueAsDouble() * 2. * Math.PI;
  }
}


class DriveEncoder {
  private final RelativeEncoder m_encoder;
  private double m_unitsPerRotation;

  public DriveEncoder (SparkMax driveMotor, double unitsPerRotation) {
    m_encoder = driveMotor.getEncoder();
    m_unitsPerRotation = unitsPerRotation;
  }

  public void reset() {
    m_encoder.setPosition(0.);
  }

  public double getPosition() {
    return m_encoder.getPosition() * m_unitsPerRotation;
  }

  public double getVelocity() {
    return m_encoder.getVelocity() * m_unitsPerRotation / 60;
  }
}

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final SparkMaxConfig m_driveConfig;
  private final SparkMaxConfig m_turnConfig;

  private final TurningEncoder m_turningEncoder;
  private final DriveEncoder m_driveEncoder;

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.P_MODULE_TURNING_CONTROLLER,
          ModuleConstants.I_MODULE_TURNING_CONTROLLER,
          ModuleConstants.D_MODULE_TURNING_CONTROLLER,
          new TrapezoidProfile.Constraints(
              ModuleConstants.MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND,
              ModuleConstants.MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQUARED));

  public SwerveModule(
    int driveMotorID,
    int turningMotorID,
    int turningEncoderID,
    double MagnetOffset) {
      
      
      m_driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
      m_driveConfig = new SparkMaxConfig();
      m_driveConfig.smartCurrentLimit(40);
      m_driveConfig.closedLoopRampRate(0.8);
      /*m_driveMotor.setPeriodicFramePeriod(
        SparkLowLevel.PeriodicFrame.kStatus0, 100 );*/
      /*m_driveMotor.setPeriodicFramePeriod(
        SparkLowLevel.PeriodicFrame.kStatus1, 20);*/
      /*m_driveMotor.setPeriodicFramePeriod(
        SparkLowLevel.PeriodicFrame.kStatus2, 20);*/
      m_driveConfig.idleMode(IdleMode.kBrake);
      m_driveMotor.configure(m_driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

      m_turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);
      m_turnConfig = new SparkMaxConfig();
      m_turnConfig.smartCurrentLimit(25);
      m_turnConfig.closedLoopRampRate(0.8);
      /*m_turningMotor.setPeriodicFramePeriod(
        SparkLowLevel.PeriodicFrame.kStatus0, 100);
      m_turningMotor.setPeriodicFramePeriod(
        SparkLowLevel.PeriodicFrame.kStatus1, 20);
      m_turningMotor.setPeriodicFramePeriod(
        SparkLowLevel.PeriodicFrame.kStatus2, 20); */
      m_turnConfig.idleMode(IdleMode.kBrake);
      m_turningMotor.configure(m_turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
          
      m_driveEncoder = new DriveEncoder(m_driveMotor,
                                     ModuleConstants.DRIVE_ENCODER_DISTANCE_PER_ROTATION);
          
      m_turningEncoder = new TurningEncoder(turningEncoderID, MagnetOffset);
          
      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(),
                 new Rotation2d(m_turningEncoder.getPositionRadians()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(),
                 new Rotation2d(m_turningEncoder.getPositionRadians()));
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = 
      SwerveModuleState.optimize(desiredState,
      new Rotation2d(m_turningEncoder.getPositionRadians()));

    final double driveOutput = state.speedMetersPerSecond /
      DriveConstants.MAX_SPEED_METERS_PER_SECOND;

    final double turnOutput =
      m_turningPIDController.calculate(m_turningEncoder.getPositionRadians(),
        state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  public void resetEncoders() {
    m_driveEncoder.reset();
  }
}