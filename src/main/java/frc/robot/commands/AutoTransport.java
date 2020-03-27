package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TransportSubsystem;

public class AutoTransport extends CommandBase {

    private TransportSubsystem mTransportSubsystem;
    private boolean[] mWasBall;
    private boolean mWasNoBall;
    private BooleanSupplier mOverride;

    public AutoTransport(TransportSubsystem transportSubsystem, BooleanSupplier override) {
        mTransportSubsystem = transportSubsystem;
        mOverride = override;

        // Require TransportSubsystem
        addRequirements(mTransportSubsystem);
        mWasBall = new boolean[Constants.Transport.kDebounceSize];
    }

    @Override
    public void initialize() {
        for (int i = 0; i < mWasBall.length; i++) {
            mWasBall[i] = false;
        }
        mWasNoBall = true;
    }

    @Override
    public void execute() {
        /*
            noBall - There was no ball in the last specified number of loops(debounced)
            mWasNoBall - The value of noBall the last loop(debounced)
            ball[] - An array of the detection of balls for this loop(not debounced)
            mWasBall[] - An array of the detection of balls for the last debounce cycles in the past loop(not debounced)
        */
        boolean noBall = true;
        boolean[] ball = mTransportSubsystem.isBall();
        boolean shooterBall = ball[1];
        // If the override is pressed, pretend there is no ball in front of the shooter
        if(mOverride.getAsBoolean()) {
            shooterBall = false;
        }
        // If the motion magic profile running on the transport is finished run the transport
        if(mTransportSubsystem.motionMagicDone() && !shooterBall) {
            // If there is a ball and there is no ball by the shooter, run the transport
            mTransportSubsystem.setVelocity(ball[0] ? Constants.Transport.kVelocity : 0);
        }
        if(shooterBall) {
            mTransportSubsystem.setVelocity(0);
        }
        // Move the ball detections one slot down and replace the first slot with the newest detection
        for (int i = mWasBall.length - 1; i > 0; i--) {
            mWasBall[i] = mWasBall[i - 1];
        }
        mWasBall[0] = ball[0];
        // Checks if there was a ball detected in past loops
        for (int i = 0; i < mWasBall.length; i++) {
            if (mWasBall[i]) {
                noBall = false;
            }
        }
        // Checks if there was a ball last loop but there is no longer a ball
        boolean ballGone = noBall && !mWasNoBall;
        // If the ball is gone and there is no ball by the shooter
        if(ballGone && !shooterBall) {
            // Move the transport to reduce jamming
            mTransportSubsystem.incrementPosition(Constants.Transport.kMargin);
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