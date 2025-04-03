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

import java.util.Currency;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class CenterCoral extends Command{
    private final DriveTrain m_DriveTrain;
    private final CoralIntake m_CoralIntake;
    private final AlgaeIntake m_AlgaeIntake;
    private final Timer m_Timer;

    public CenterCoral(DriveTrain driveTrain, CoralIntake coralIntake, AlgaeIntake algaeIntake) {
        m_DriveTrain = driveTrain;
        m_AlgaeIntake = algaeIntake;
        m_CoralIntake = coralIntake;
        m_Timer = new Timer();

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        m_Timer.restart();
    }

    @Override
    public void execute() {
        int currentTime = (int) Math.floor(m_Timer.get());
        if (currentTime <= 3) {
            m_DriveTrain.drive(0.1, 0, 0, false);
            States.mElevatorState = States.ElevatorStates.l4;
        } else if (currentTime <= 5) {
            m_DriveTrain.stop();
            m_CoralIntake.setSpeed(0.5);
        } else if (currentTime <= 6) {
            m_DriveTrain.drive(-0.1, 0.05, 0, false);
        } else if (currentTime <= 7) {
            States.mElevatorState = States.ElevatorStates.aL;
            m_AlgaeIntake.setAngle(Value.kReverse);
            m_AlgaeIntake.setSpeed(-1.0);
        } else if (currentTime <= 9) {
            m_DriveTrain.drive(0.1, 0, 0, false);
        }
        
    }
    
    @Override
    public boolean isFinished() {
        if (m_Timer.get() > 12.0) {
            return true;
        }
        return false;
    }
}

// public class CenterCoral extends SequentialCommandGroup {
//     public CenterCoral(DriveTrain driveTrain, CoralIntake coralIntake, AlgaeIntake algaeIntake) {
//         addCommands(
//             new InstantCommand(() -> driveTrain.drive(0.1, 0, 0, false)),
//             new InstantCommand(() -> States.mElevatorState = ElevatorStates.l4),
//             new WaitCommand(3.0),
//             new InstantCommand(() -> driveTrain.stop()),
//             new InstantCommand(() -> coralIntake.setSpeed(1.0)),
//             new WaitCommand(2.0),
//             new InstantCommand(() -> coralIntake.setSpeed(0)),
//             new InstantCommand(() -> driveTrain.drive(-0.1, -0.05, 0, false)),
//             new WaitCommand(1.0),
//             new InstantCommand(() -> driveTrain.stop()),
//             new InstantCommand(() -> States.mElevatorState = ElevatorStates.aL),
//             new InstantCommand(() -> algaeIntake.setAngle(Value.kReverse)),
//             new InstantCommand(() -> algaeIntake.setSpeed(-1.0)),
//             new WaitCommand(1.0),
//             new InstantCommand(() -> driveTrain.drive(0.1, 0, 0, false)),
//             new WaitCommand(1.2),
//             new InstantCommand(() -> driveTrain.stop())
//         );
//     }
// }
