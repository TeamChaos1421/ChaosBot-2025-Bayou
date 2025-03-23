// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  private final SwerveModule m_frontLeft =
    new SwerveModule(
      DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURNING_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURNING_ENCODER_ID,
      DriveConstants.FRONT_LEFT_MAGNET_OFFSET);

  private final SwerveModule m_rearLeft =
    new SwerveModule(
      DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURNING_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURNING_ENCODER_ID,
      DriveConstants.REAR_LEFT_MAGNET_OFFSET);

  private final SwerveModule m_frontRight =
    new SwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURNING_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURNING_ENCODER_ID,
      DriveConstants.FRONT_RIGHT_MAGNET_OFFSET);

  private final SwerveModule m_rearRight =
    new SwerveModule(
      DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURNING_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURNING_ENCODER_ID,
      DriveConstants.REAR_RIGHT_MAGNET_OFFSET);

  public AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  Limelight limelight;

  MedianFilter limelightXFilter;
  MedianFilter limelightYFilter;
  MedianFilter limelightYawFilter;

  double compositeLatency;
  Pose2d compositeVisionPose;

  boolean isVisionValid;

  public SwerveDriveOdometry swerveOdometry;
  public RobotConfig config = new RobotConfig(
    Constants.AutoConstants.ROBOT_MASS_KG,
    Constants.AutoConstants.ROBOT_MOI,
    Constants.AutoConstants.moduleConfig,
    Constants.DriveConstants.TRACK_WIDTH);

  /** Creates a new DriveTrain. */
  public DriveTrain(Limelight l) {

    limelight = l;

    limelightXFilter = new MedianFilter(3);
    limelightYFilter = new MedianFilter(3);
    limelightYawFilter = new MedianFilter(3);

    compositeLatency = 0;
    compositeVisionPose = new Pose2d();

    isVisionValid = true;

    swerveOdometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, getRotation2d(), get_positions());

    AutoBuilder.configure(
      () -> swerveOdometry.getPoseMeters(), // Robot pose supplier
      (Pose2d pose) -> swerveOdometry.resetPosition(getRotation2d(), get_positions(), pose), // First used to be getHeading(), // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        Constants.AutoConstants.translationPID, // Translation PID constants
        Constants.AutoConstants.rotationPID // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro", getHeading());
    // This method will be called once per scheduler run
  }

  public void drive(double xSpeed, double ySpeed, double rot,
    boolean fieldRelative)
  {
    double m_xSpeed = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double m_ySpeed = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double m_rot = rot * DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    var swerveModuleStates =
      DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        (fieldRelative)
          //   Teleop otherwise auto
          ? ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rot,
              getRotation2d())
          : new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rot));

    setModuleStates(swerveModuleStates);
  }

  public void driveAuto(ChassisSpeeds speeds) {
    SwerveModuleState[] states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public SwerveModuleState[] getModuleStates() {
    var myModuleStates = new SwerveModuleState[4];
    myModuleStates[0] = m_frontLeft.getState();
    myModuleStates[1] = m_rearLeft.getState();
    myModuleStates[2] = m_rearRight.getState();
    myModuleStates[3] = m_frontRight.getState();
    return myModuleStates;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_rearLeft.setDesiredState(desiredStates[1]);
    m_rearRight.setDesiredState(desiredStates[2]);
    m_frontRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void stop() {
    var swerveModuleStates =
      DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(0., 0., 0.));
    setModuleStates(swerveModuleStates);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-m_gyro.getYaw());
  }

  public double getHeading() {
    return -m_gyro.getYaw();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public SwerveModulePosition[] get_positions() {
    SwerveModulePosition[] m_positions = {m_frontLeft.getPosition(),m_rearLeft.getPosition(),
    m_rearRight.getPosition(),m_frontRight.getPosition()};
    return m_positions;
  }
}
