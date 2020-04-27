package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enums.ClimbState;
import frc.robot.subsystems.ClimbSubsystem;


public class Climb extends CommandBase {
    private ClimbSubsystem mClimbSubsystem;
    private ClimbState mState;


    /**
     * Creates a new climb system
     * @param climbSubsystem The climbSubsystem to use
     * @param state The state of the system(0=retract, 1=vent, 2=extend)
     */
    public Climb(ClimbSubsystem climbSubsystem, ClimbState state) {
        mClimbSubsystem = climbSubsystem;
        mState = state;

        // Require ClimbSubsystem
        addRequirements(mClimbSubsystem);
    }

    @Override
    public void initialize() {
        mClimbSubsystem.setState(mState);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        mClimbSubsystem.setState(ClimbState.RETRACTED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}