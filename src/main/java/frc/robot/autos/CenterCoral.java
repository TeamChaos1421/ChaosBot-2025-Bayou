package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AlignAprilTag;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.States;
import frc.robot.subsystems.States.ElevatorStates;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class CenterCoral extends SequentialCommandGroup {
    public CenterCoral(DriveTrain driveTrain, CoralIntake coralIntake, AlgaeIntake algaeIntake) {
        addCommands(
            new InstantCommand(() -> driveTrain.drive(0.1, 0, 0, false)),
            new InstantCommand(() -> States.mElevatorState = ElevatorStates.l4),
            new WaitCommand(3.0),
            new InstantCommand(() -> coralIntake.setSpeed(1.0)),
            new InstantCommand(() -> driveTrain.drive(-0.1, -0.05, 0, false)),
            new WaitCommand(1.0),
            new InstantCommand(() -> driveTrain.stop()),
            new InstantCommand(() -> States.mElevatorState = ElevatorStates.aL),
            new InstantCommand(() -> algaeIntake.setAngle(Value.kReverse)),
            new InstantCommand(() -> algaeIntake.setSpeed(-1.0)),
            new WaitCommand(1.0),
            new InstantCommand(() -> driveTrain.drive(0.1, 0, 0, false)),
            new WaitCommand(1.2),
            new InstantCommand(() -> driveTrain.stop())
        );
    }
}
