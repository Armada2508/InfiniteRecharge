package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TransportSubsystem;

public class AutoTransport extends CommandBase {

    private TransportSubsystem mTransportSubsystem;
    private boolean[] mWasBall;
    private boolean mWasNoBall;

    public AutoTransport(TransportSubsystem transportSubsystem) {
        mTransportSubsystem = transportSubsystem;

        // Require TransportSubsystem
        addRequirements(mTransportSubsystem);
        mWasBall = new boolean[5];
    }

    @Override
    public void initialize() {
        for (int i = 0; i < mWasBall.length; i++) {
            mWasBall[i] = false;
        }
    }

    @Override
    public void execute() {
        boolean noBall = true;
        boolean[] ball = mTransportSubsystem.isBall();
        if(mTransportSubsystem.motionMagicDone()) {
            mTransportSubsystem.setVelocity(ball[0] ? Constants.Transport.kTransportVelocity : 0);
        }
        for (int i = mWasBall.length - 1; i > 0; i--) {
            mWasBall[i] = mWasBall[i - 1];
        }
        mWasBall[0] = ball[0];
        for (int i = 0; i < mWasBall.length; i++) {
            if (mWasBall[i]) {
                noBall = false;
            }
        }
        if(noBall && !mWasNoBall) {
            mTransportSubsystem.incrementPosition(5);
        }
        mWasNoBall = noBall;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}