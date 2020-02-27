package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends CommandBase {
    private ClimbSubsystem mClimbSubsystem;
    private int mState;


    /**
     * Creates a new climb system
     * @param climbSubsystem The climbSubsystem to use
     * @param state The state of the system(0=retract, 1=vent, 2=extend)
     */
    public Climb(ClimbSubsystem climbSubsystem, int state) {
        mClimbSubsystem = climbSubsystem;
        mState = state;

        // Require ClimbSubsystem
        addRequirements(mClimbSubsystem);
    }

    @Override
    public void initialize() {
        switch (mState) {
            case 0:
                mClimbSubsystem.retract();
                break;
            case 1:
                mClimbSubsystem.vent();
                break;
            case 2:
                mClimbSubsystem.extend();
                break;
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}