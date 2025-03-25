package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class AlignAprilTag extends Command {
    DriveTrain driveTrain;
    String target;

    double Ltx;
    double Lta;

    double Rtx;
    double Rta;

    public AlignAprilTag(DriveTrain dt, String tgt) {
        driveTrain = dt;
        target = tgt;
        addRequirements(driveTrain);
    }
    
    @Override
    public void execute() {
        Ltx = LimelightHelpers.getTX("limelight-left");
        Lta = LimelightHelpers.getTA("limelight-left");

        Rtx = LimelightHelpers.getTX("limelight-right");
        Rta = LimelightHelpers.getTA("limelight-right");

        if (target == DriveConstants.LEFT_TARGET) {
            driveTrain.drive(Rtx * 0.05, (100 - Rta) * 0.03, 0, false);
        } else {
            driveTrain.drive(Ltx * 0.05, (100 - Lta) * 0.03, 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        if (target == DriveConstants.LEFT_TARGET && Math.abs(Rtx) < 0.1 && Rta > 0.7) {
            return true;
        } else if (Math.abs(Ltx) < 0.1 && Lta > 0.7) {
            return true;
        } else {
            return false;
        }
    }
}
