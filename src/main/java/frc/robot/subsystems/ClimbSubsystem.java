package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    
    private Solenoid m_lTop;
    private Solenoid m_lBottom;
    private Solenoid m_rTop;
    private Solenoid m_rBottom;


    public ClimbSubsystem() {
        m_lTop = new Solenoid(Constants.kLeftClimbTop);
        m_lBottom = new Solenoid(Constants.kLeftClimbBottom);
        m_rTop = new Solenoid(Constants.kRightClimbTop);
        m_rBottom = new Solenoid(Constants.kRightClimbBottom);
    }

    @Override
    public void periodic() {
        
    }

    public void extend() {
        m_lTop.set(true);
        m_lBottom.set(true);
        m_rTop.set(true);
        m_rBottom.set(true);
    }

    public void vent() {
        m_lTop.set(true);
        m_lBottom.set(false);
        m_rTop.set(true);
        m_rBottom.set(false);
        
    }

    public void retract() {
        m_lTop.set(false);
        m_lBottom.set(false);
        m_rTop.set(false);
        m_rBottom.set(false);
    }
}