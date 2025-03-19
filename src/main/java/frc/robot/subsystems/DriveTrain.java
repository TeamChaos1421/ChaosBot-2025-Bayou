// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorIDs;
import frc.robot.Constants.VisionConstants;

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

  public final Pigeon2 m_gyro = new Pigeon2(SensorIDs.GYRO, "1912pizzavore");

  Limelight limelight;

  public boolean fieldRelative;

  MedianFilter limelightXFilter;
  MedianFilter limelightYFilter;
  MedianFilter limelightYawFilter;

  double compositeLatency;
  Pose2d compositeVisionPose;

  boolean isVisionValid;

  /** Creates a new DriveTrain. */
  public DriveTrain(Limelight l) {

    limelight = l;
    fieldRelative = true;

    limelightXFilter = new MedianFilter(3);
    limelightYFilter = new MedianFilter(3);
    limelightYawFilter = new MedianFilter(3);

    compositeLatency = 0;
    compositeVisionPose = new Pose2d();

    isVisionValid = true;
  }

  @Override
  public void periodic() {
    processFrame();
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
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
  }

  public double getHeading() {
    return m_gyro.getYaw().getValueAsDouble();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public SwerveModulePosition[] get_positions() {
    SwerveModulePosition[] m_positions = {m_frontLeft.getPosition(),m_rearLeft.getPosition(),
    m_rearRight.getPosition(),m_frontRight.getPosition()};
    return m_positions;
  }

  public void processFrame() {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double totalArea = 0;
    isVisionValid = false;

    if (limelight.getTagId() > 0) {
      if (limelight.getTargetArea() > VisionConstants.TARGET_AREA_THRESHHOLD) {
        totalArea += limelight.getTargetArea();
        x += limelight.getBotPose2d().getX() * limelight.getTargetArea();
        y += limelight.getBotPose2d().getY() * limelight.getTargetArea();
        yaw += limelight.getBotPose2d().getRotation().getDegrees() * limelight.getTargetArea();
        compositeLatency += limelight.getLatency();
      }
    } 

    if (totalArea < VisionConstants.TOTAL_TARGET_AREA_THRESHHOLD) {
      isVisionValid = false;
      compositeLatency = 0;
    } else {
      isVisionValid = true;
      x /= totalArea;
      y /= totalArea;
      yaw /= totalArea;
      compositeLatency /= totalArea;

      compositeVisionPose = new Pose2d(
        limelightXFilter.calculate(x),
        limelightYFilter.calculate(y),
        Rotation2d.fromDegrees(limelightYawFilter.calculate(yaw))
      );
    }

  }
}
