package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransportSubsystem;

public class AutoTransport extends CommandBase {

    private TransportSubsystem mTransportSubsystem;

    public AutoTransport(TransportSubsystem transportSubsystem) {
        mTransportSubsystem = transportSubsystem;

        // Require TransportSubsystem
        addRequirements(mTransportSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}