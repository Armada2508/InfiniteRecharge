package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.ClimbState;

public class ClimbSubsystem extends SubsystemBase {
    
    private Solenoid mLTop;
    private Solenoid mLBottom;
    private Solenoid mRTop;
    private Solenoid mRBottom;
    private ClimbState mState;


    public ClimbSubsystem() {
        mLTop = new Solenoid(Constants.Climb.kLeftTop);
        mLBottom = new Solenoid(Constants.Climb.kLeftBottom);
        mRTop = new Solenoid(Constants.Climb.kRightTop);
        mRBottom = new Solenoid(Constants.Climb.kRightBottom);
    }

    @Override
    public void periodic() {
        
    }

    public void setState(ClimbState state) {
        switch (state) {
            case EXTENDED:
                    extend();
                break;
            case RETRACTED:
                    retract();
                break;
            case VENTED:
                    vent();
                break;
            default:
                    retract();
                break;
        }
        mState = state;
    }

    private void extend() {
        mLTop.set(true);
        mLBottom.set(true);
        mRTop.set(true);
        mRBottom.set(true);
    }

    private void vent() {
        mLTop.set(true);
        mLBottom.set(false);
        mRTop.set(true);
        mRBottom.set(false);
        
    }

    private void retract() {
        mLTop.set(false);
        mLBottom.set(false);
        mRTop.set(false);
        mRBottom.set(false);
    }

    public ClimbState getState() {
        if(DriverStation.getInstance().isDisabled()) {
            return ClimbState.RETRACTED;
        }
        return mState;
    }

    public boolean isExtended() {
        if(DriverStation.getInstance().isDisabled()) {
            return false;
        }
        return mState == ClimbState.EXTENDED;
    }

    public boolean isRetracted() {
        if(DriverStation.getInstance().isDisabled()) {
            return true;
        }
        return mState == ClimbState.RETRACTED;
    }

    public boolean isVented() {
        if(DriverStation.getInstance().isDisabled()) {
            return false;
        }
        return mState == ClimbState.VENTED;
    }
}