package frc.lib.drive;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * Calculates Swerve Module States, explanation of calculations can be found here: https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
 */
public class SwerveDrive {

    private Translation2d[] mWheelPositions;

    /**
     * Creates a new SwerveDrive object
     * @param wheelPositions The positions of the wheels relative to the center of the robot
     */
    public SwerveDrive(Translation2d... wheelPositions) {
        mWheelPositions = wheelPositions.clone();
    }

    /**
     * Calculates Swerve Module states for a given movement 
     * @param tX The translational x movement(left/right; m/s)
     * @param tY The translational y movement(forwards/backwards, m/s)
     * @param twist The rotational(twist) movement(CW positive, rad/s)
     * @param maxVelocity The maximum velocity of the swerve modules
     * @return The state of each of the swerve modules
     */
    public SwerveModuleState[] calculate(double tX, double tY, double twist, double maxVelocity) {
        SwerveModuleState[] mModuleStates = new SwerveModuleState[mWheelPositions.length];
        for (int i = 0; i < mWheelPositions.length; i++) {
            double rX = twist * mWheelPositions[i].getY();
            double rY = -twist * mWheelPositions[i].getX();
            Translation2d velocity = new Translation2d(tX + rX, tY + rY);
            Rotation2d angle = new Rotation2d(Math.IEEEremainder(Math.atan2(velocity.getY(), velocity.getX()), 2*Math.PI));
            mModuleStates[i] = new SwerveModuleState(velocity.getNorm(), angle);
        }
        double maxCalculatedVelocity = Double.MIN_VALUE;
        for (int i = 0; i < mModuleStates.length; i++) {
            maxCalculatedVelocity = Math.max(maxCalculatedVelocity, mModuleStates[i].speedMetersPerSecond);
        }
        if(maxCalculatedVelocity > maxVelocity) {
            for (int i = 0; i < mModuleStates.length; i++) {
                mModuleStates[i].speedMetersPerSecond = mModuleStates[i].speedMetersPerSecond * maxVelocity / maxCalculatedVelocity;
            }
        }
        return mModuleStates;
    }


}