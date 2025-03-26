package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveForward extends SequentialCommandGroup {
    public DriveForward(DriveTrain driveTrain) {
        addCommands(
            new InstantCommand(() -> driveTrain.drive(0.1, 0, 0, false)),
            new WaitCommand(5.0),
            new InstantCommand(() -> driveTrain.stop())
        );
    }
}

// public class DriveForward extends Command{
//     private final DriveTrain m_DriveTrain;
//     private final Timer m_Timer;

//     public DriveForward(DriveTrain driveTrain) {
//         m_DriveTrain = driveTrain;
//         m_Timer = new Timer();

//         addRequirements(driveTrain);
//     }

//     @Override
//     public void initialize() {
//         m_Timer.restart();
//     }

//     @Override
//     public void execute() {
//         m_DriveTrain.drive(0.1, 0, 0, false);
//     }
    
//     @Override
//     public boolean isFinished() {
//         if (m_Timer.get() > 5.0) {
//             return true;
//         }
//         return false;
//     }
// }
