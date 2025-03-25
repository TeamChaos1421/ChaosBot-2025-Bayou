package frc.robot.subsystems;

public class States {
    public static enum ElevatorStates {
        intake(1.0), l1(1.0), l2(1.0), aL(1.0), l3(0.5), aH(0.5), l4(0.3);

        public final double driveSpeed;
        ElevatorStates(double driveSpeed) {
            this.driveSpeed = driveSpeed;
        }
    }

    public static Boolean mElevatorToggle = false;
    public static ElevatorStates mElevatorState = ElevatorStates.intake;
    public static Boolean mFieldOriented = true;
}
