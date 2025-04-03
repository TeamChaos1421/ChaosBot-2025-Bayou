package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Leds extends SubsystemBase{
    public Solenoid whiteStrips;
    public Timer mTimer;

    public Leds() {
        mTimer = new Timer();
        whiteStrips = new Solenoid(Constants.kPCMCANId,PneumaticsModuleType.CTREPCM, 4);

        mTimer.reset();
        mTimer.start();
    }

    public void Blink() {
        double delay = mTimer.get() - Math.floor(mTimer.get());
        if (delay > 0.5) {
            whiteStrips.set(true);
        } else {
            whiteStrips.set(false);
        }
    }
}
