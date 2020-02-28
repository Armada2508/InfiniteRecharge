package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    
    private Solenoid mLTop;
    private Solenoid mLBottom;
    private Solenoid mRTop;
    private Solenoid mRBottom;


    public ClimbSubsystem() {
        mLTop = new Solenoid(Constants.Climb.kLeftClimbTop);
        mLBottom = new Solenoid(Constants.Climb.kLeftClimbBottom);
        mRTop = new Solenoid(Constants.Climb.kRightClimbTop);
        mRBottom = new Solenoid(Constants.Climb.kRightClimbBottom);
    }

    @Override
    public void periodic() {
        
    }

    public void extend() {
        mLTop.set(true);
        mLBottom.set(true);
        mRTop.set(true);
        mRBottom.set(true);
    }

    public void vent() {
        mLTop.set(true);
        mLBottom.set(false);
        mRTop.set(true);
        mRBottom.set(false);
        
    }

    public void retract() {
        mLTop.set(false);
        mLBottom.set(false);
        mRTop.set(false);
        mRBottom.set(false);
    }
}