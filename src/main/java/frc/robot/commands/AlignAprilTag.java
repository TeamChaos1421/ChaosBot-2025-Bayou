package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class AlignAprilTag extends Command {
    DriveTrain driveTrain;
    String target;
    String limelightName;

    boolean tv;
    double tx;
    double ta;
    int id;

    double[] targetAngles;

    public AlignAprilTag(DriveTrain dt, String tgt, boolean isAuto) {
        driveTrain = dt;
        target = tgt;
        limelightName = (target == DriveConstants.LEFT_TARGET ? "limelight-right" : "limelight-left");
        targetAngles = (isAuto ? DriveConstants.InvertedTagAngles : DriveConstants.aprilTagAngles);
        addRequirements(driveTrain);
    }
    
    @Override
    public void execute() {
        tv = LimelightHelpers.getTV(limelightName);
        tx = LimelightHelpers.getTX(limelightName);
        ta = LimelightHelpers.getTA(limelightName);
        id = (int) Math.round(LimelightHelpers.getFiducialID(limelightName));

        if (tv && id > 0) {
            driveTrain.drive(
                (12 - ta) * 0.01 + 0.01, // Forward and Back
                -tx * 0.03, // Left and Right
                (targetAngles[id] - (driveTrain.getHeading())) * 0.03, // Rotation
                false
            );
        }
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(tx) < 1 && Math.abs(ta - 12) < 1) {
            return true;
        } else {
            return false;
        }
    }
}
