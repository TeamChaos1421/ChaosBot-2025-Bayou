// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverController;
import frc.robot.Constants.OperatorController;
import frc.robot.autos.CenterCoral;
import frc.robot.autos.DriveForward;
import frc.robot.Constants.ButtonPanel;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final Climber s_Climber = new Climber();
  private final Elevator s_Elevator = new Elevator();
  private final CoralIntake s_CoralIntake = new CoralIntake();
  private final AlgaeIntake s_AlgaeIntake = new AlgaeIntake();
  private final Pneumatics s_Pneumatics = new Pneumatics();
  private final Leds s_Leds = new Leds();

  public final DigitalInput elevatorSwitch = new DigitalInput(0);
  public final DigitalInput testSwitch = new DigitalInput(1);
  private final Trigger resetElevator = new Trigger(elevatorSwitch::get);

  private final ZeroHeading zeroHeading = new ZeroHeading(driveTrain);

  private final SendableChooser<Command> autoChooser;
  private final Command a_DriveForward = new DriveForward(driveTrain);
  private final Command a_CenterCoral = new CenterCoral(driveTrain, s_CoralIntake, s_AlgaeIntake);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(DriverController.DRIVER_JOYSTICK);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorController.OPERATOR_JOYSTICK);
  private final CommandJoystick buttonPanel = new CommandJoystick(ButtonPanel.PANEL_JOYSTICK);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Pneumatics.Init();
    CameraServer.startAutomaticCapture(0);
    zeroHeading.addRequirements(driveTrain);

    driveTrain.setDefaultCommand(
      new RunCommand(() -> driveTrain.drive(
        -MathUtil.applyDeadband(driverController.getLeftY(), Constants.DriverController.XY_DEADBAND) * States.mElevatorState.driveSpeed, 
        -MathUtil.applyDeadband(driverController.getLeftX(), Constants.DriverController.XY_DEADBAND) * States.mElevatorState.driveSpeed, 
        -MathUtil.applyDeadband(driverController.getRightX(), Constants.DriverController.ROT_DEADBAND) * States.mElevatorState.driveSpeed, 
        States.mFieldOriented), 
      driveTrain));

    s_Elevator.setDefaultCommand(
        new ElevatorCommand(
            s_Elevator, 
            () -> (-MathUtil.applyDeadband(operatorController.getLeftY(), Constants.OperatorController.XY_DEADBAND))
        )
    );

    s_CoralIntake.setDefaultCommand(
        new CoralCommand(
            s_CoralIntake,
            () -> operatorController.getHID().getRawButton(XboxController.Button.kA.value),
            () -> operatorController.getHID().getRawButton(XboxController.Button.kB.value)
        )
    );

    s_AlgaeIntake.setDefaultCommand(
        new AlgaeCommand(
            s_AlgaeIntake,
            () -> operatorController.getHID().getRawButton(XboxController.Button.kX.value),
            () -> operatorController.getHID().getRawButton(XboxController.Button.kY.value)
        )
    );

    s_Climber.setDefaultCommand(Commands.run(
        () -> {
          if (States.mElevatorToggle == false) {
            s_Climber.setSpeed(-buttonPanel.getHID().getRawAxis(Joystick.kDefaultYChannel));
          } else {
            s_Climber.setSpeed(-MathUtil.applyDeadband(operatorController.getRightY(), Constants.OperatorController.XY_DEADBAND));
          }
          
        },
        s_Climber
    ));

    s_Leds.setDefaultCommand(
      (
        Commands.run(() -> {
          if(States.mElevatorToggle) {
            s_Leds.whiteStrips.set(true);
          } else {
            s_Leds.Blink();
          }
        }, s_Leds).ignoringDisable(true)));

    // Configure the trigger bindings
    configureBindings();

    //Auto chooser
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Drive Forward", a_DriveForward);
    autoChooser.addOption("Center Coral", a_CenterCoral);
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    resetElevator.onFalse(new InstantCommand(() -> s_Elevator.zeroPos()));
    // DRIVER XBOX CONTROLLER
    driverController.y().onTrue(zeroHeading);
    driverController.leftBumper()
      .onTrue(new InstantCommand(() -> States.mFieldOriented = false))
      .onFalse(new InstantCommand(() -> States.mFieldOriented = true));
    driverController.pov(90).whileTrue(new AlignAprilTag(driveTrain, DriveConstants.RIGHT_TARGET, false));
    driverController.pov(270).whileTrue(new AlignAprilTag(driveTrain, DriveConstants.LEFT_TARGET, false));

    // CODRIVER XBOX CONTROLLER
    operatorController.start().onTrue(
        new InstantCommand(() -> {
            States.mElevatorToggle = !States.mElevatorToggle;
        })
    );
    operatorController.back().onTrue(new InstantCommand(() -> s_Elevator.zeroPos()));
    operatorController.pov(0).onTrue(new InstantCommand(() -> s_AlgaeIntake.setAngle(Value.kForward)));
    operatorController.pov(180).onTrue(new InstantCommand(() -> s_AlgaeIntake.setAngle(Value.kReverse)));
    operatorController.rightBumper().onTrue(new InstantCommand(() -> {
      if(States.mElevatorToggle){s_CoralIntake.setAngle(Value.kReverse);
    }}));
    operatorController.leftBumper().onTrue(new InstantCommand(() -> {
      if(States.mElevatorToggle){s_CoralIntake.setAngle(Value.kForward);
    }}));

    // CODRIVER BUTTON PANEL
    buttonPanel.button(ButtonPanel.setTargetIntake).onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.intake;}));
    buttonPanel.button(ButtonPanel.setTargetL1).onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l1;}));
    buttonPanel.button(ButtonPanel.setTargetL2).onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l2;}));
    buttonPanel.button(ButtonPanel.setTargetAL).onTrue(new InstantCommand(() -> {
        States.mElevatorState = States.ElevatorStates.aL;
        s_AlgaeIntake.setAngle(Value.kReverse);
    }));
    buttonPanel.button(ButtonPanel.setTargetL3).onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l3;}));
    buttonPanel.button(ButtonPanel.setTargetAH).onTrue(new InstantCommand(() -> {
        States.mElevatorState = States.ElevatorStates.aH;
        s_AlgaeIntake.setAngle(Value.kReverse);
    }));
    buttonPanel.button(ButtonPanel.setTargetL4).onTrue(new InstantCommand(() -> {States.mElevatorState = States.ElevatorStates.l4;}));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
