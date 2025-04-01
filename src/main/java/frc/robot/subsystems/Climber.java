package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private SparkMax mClimberMotor;
    private Encoder mClimberEncoder;

    public Climber() {
        mClimberMotor = new SparkMax(Constants.Climber.climberCanID, SparkMax.MotorType.kBrushless);
        mClimberEncoder = new Encoder(Constants.Climber.encoderA, Constants.Climber.encoderB);
    }

    public Climber setSpeed(double speed) {
        int currentPOS = mClimberEncoder.get();
        
        if (currentPOS <= Constants.Climber.maxOut) {
            mClimberMotor.set(Math.min(speed, 0));
        } else if (currentPOS >= Constants.Climber.maxIn) {
            mClimberMotor.set(Math.max(speed, 0));
        } else {
            mClimberMotor.set(speed);
        }
        
        return this;
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Position", mClimberEncoder.get());
    }
}
