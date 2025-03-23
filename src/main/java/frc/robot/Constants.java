// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ModuleConstants {
    public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 10000.; //2 * 2 * Math.PI;
    public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQUARED = 10000.; //2 * 2 * Math.PI;

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double WHEEL_GEAR_RATIO = 6.75;
    public static final double DRIVE_ENCODER_DISTANCE_PER_ROTATION =
      (WHEEL_DIAMETER_METERS * Math.PI) / WHEEL_GEAR_RATIO;

    public static final double P_MODULE_TURNING_CONTROLLER = 0.5;
    public static final double I_MODULE_TURNING_CONTROLLER = 0.0;
    public static final double D_MODULE_TURNING_CONTROLLER = 0.0067;

  }

  public static final int climberCanID = 62;

  public static final int kPCMCANId = 1;
 
  public static final class CoralIntake {
      public static final int coralCanID = 53; 
      public static final int kDumpForward = 0;
      public static final int kDumpReverse = 1;
  }

  public static final class AlgaeIntake {
      public static final int algaeIntakeCanID = 50; 
      public static final int kDumpForward = 2;
      public static final int kDumpReverse = 3;
  }

  public static final class Elevator {
      public static final int leftElevatorID = 21;
      public static final int rightElevatorID = 22;
      public static final int elevatorEncoderID = 3;

      public static final double intakePOS = 2.5;
      public static final double l1POS = 0;
      public static final double l2POS = 10.5;
      public static final double aLPOS = 41;
      public static final double l3POS = 33;
      public static final double aHPOS = 62.5;
      public static final double l4POS = 70;

      public static final double elevatorKP = 0.035;
      public static final double elevatorKI = 0.005;
      public static final double elevatorKD = 0.0;
  }

  public static class DriveConstants {

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 52;
    public static final int FRONT_LEFT_TURNING_MOTOR_ID = 59;
    public static final int FRONT_LEFT_TURNING_ENCODER_ID = 11;
    public static final double FRONT_LEFT_MAGNET_OFFSET = -0.004638671875;

    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 56;
    public static final int REAR_LEFT_TURNING_MOTOR_ID = 61;
    public static final int REAR_LEFT_TURNING_ENCODER_ID = 13;
    public static final double REAR_LEFT_MAGNET_OFFSET = -0.245361328125;

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 55;
    public static final int FRONT_RIGHT_TURNING_MOTOR_ID = 58;
    public static final int FRONT_RIGHT_TURNING_ENCODER_ID = 10;
    public static final double FRONT_RIGHT_MAGNET_OFFSET = -0.09228515625;

    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 54;
    public static final int REAR_RIGHT_TURNING_MOTOR_ID = 60;
    public static final int REAR_RIGHT_TURNING_ENCODER_ID = 12;
    public static final double REAR_RIGHT_MAGNET_OFFSET = -0.1640625;

    public static final int GYROSCOPE_ID = 0;

    public static final double DRIVE_SPEED = 1;

    public static final double TRACK_WIDTH = Units.inchesToMeters(25);
    public static final double WHEEL_BASE = Units.inchesToMeters(24.5);
    /*public static final double FRONT_WHEEL_X = Units.inchesToMeters(6.456);
    public static final double REAR_WHEEL_X = Units.inchesToMeters(12.293);*/
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),    //FL
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),   //RL
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),  //RR
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)   //FR
            );

    public static final boolean GYRO_REVERSED = false;
    public static final double OFFSET_X_TO_ROT = 0.05;

    public static final double S_VOLTS = 1;
    public static final double V_VOLT_SECONDS_PER_METER = 0.8;
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.15;

    public static final double MAX_SPEED_METERS_PER_SECOND = 4;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2*Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;
  }

  public static class DriverController {
    public static final double XY_DEADBAND = .1;
    public static final double ROT_DEADBAND = .1;

    public static final int DRIVER_JOYSTICK = 0;
    //Controller Axes
    public static final int LEFT_Y_AXIS = 1; 
    public static final int LEFT_X_AXIS = 0; 
    public static final int RIGHT_Y_AXIS = 5; 
    public static final int RIGHT_X_AXIS = 4; 
    public static final int LEFT_TRIGGER = 2; 
    public static final int RIGHT_TRIGGER = 3;
    //Controller Buttons
    public static final int A_BUTTON = 1; 
    public static final int B_BUTTON = 2; 
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LT_BUMPER = 5;
    public static final int RT_BUMPER = 6; 
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
  }

  public static class OperatorController {
    public static final int OPERATOR_JOYSTICK = 1;
    //Controller Axes
    public static final int LEFT_Y_AXIS = 1; 
    public static final int LEFT_X_AXIS = 0; 
    public static final int RIGHT_Y_AXIS = 5; 
    public static final int RIGHT_X_AXIS = 4; 
    public static final int LEFT_TRIGGER = 2; 
    public static final int RIGHT_TRIGGER = 3;
    //Controller Buttons
    public static final int A_BUTTON = 1; 
    public static final int B_BUTTON = 2; 
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LT_BUMPER = 5;
    public static final int RT_BUMPER = 6; 
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
    public static final int LEFT_JOYSTICK = 9;
  }

  public static class ButtonPanel {
    public static final int PANEL_JOYSTICK = 2;
  }

  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    //TODO: Must be tuned to specific robot
    public static final PIDConstants translationPID = new PIDConstants(5, 0, 0);/// All values weree 0. Chnaged to see if the robot moves in automous
    public static final PIDConstants rotationPID = new PIDConstants(5, 0, 0);

    //TODO: Must be tuned to specific robot
    public static final double ROBOT_MASS_KG = 56.699;
    public static final double ROBOT_MOI = 4.5;
    public static final double WHEEL_COF = 1.2;

    public static final ModuleConfig moduleConfig = new ModuleConfig(
            2,
            DriveConstants.MAX_SPEED_METERS_PER_SECOND,
            Constants.AutoConstants.WHEEL_COF,
            DCMotor.getNEO(1).withReduction(6.75),
            60,
          1);

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}