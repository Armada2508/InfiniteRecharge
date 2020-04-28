package frc.lib.controller;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.lib.util.Util;

public class FieldOrientedController {
    private Supplier<Vector2d> mVelocity;
    private Supplier<Rotation2d> mHeading;
    private double mMaxVelocity;
    private PIDController mTurnController;
    private DifferentialDriveKinematics mKinematics;

    
    /**
     * Creates a new FieldOrientedController Object
     * @param maxVelocity The maximum velocity of the robot
     * @param turnController The PID controller used for turning
     * @param trackWidth The width of the drivetrain
     */
    public FieldOrientedController(double maxVelocity, PIDController turnController, double trackWidth) {
        mMaxVelocity = maxVelocity;
        mTurnController = turnController;
        mKinematics = new DifferentialDriveKinematics(trackWidth);
    }

    /**
     * Creates a new FieldOrientedController Object
     * @param velocity The global velocity of the robot(+X is away from alliance wall, +Y is left facing opposing alliance)
     * @param heading The heading of the robot in radians
     * @param maxVelocity The maximum velocity of the robot
     * @param turnController The PID controller used for turning
     * @param trackWidth The width of the drivetrain
     */
    public FieldOrientedController(Supplier<Vector2d> velocity, Supplier<Rotation2d> heading, double maxVelocity, PIDController turnController, double trackWidth) {
        mVelocity = velocity;
        mHeading = heading;
        mMaxVelocity = maxVelocity;
        mTurnController = turnController;
        mKinematics = new DifferentialDriveKinematics(trackWidth);
    }

    /**
     * @return The wheel speeds for the robot
     */
    public DifferentialDriveWheelSpeeds calculate() {
        if(mVelocity == null || mHeading == null) {
            return new DifferentialDriveWheelSpeeds(0.0, 0.0);
        }
        return calculate(mVelocity.get(), mHeading.get());
    }
    
    /**
     * Resets the controller
     */
    public void reset() {
        mTurnController.reset();
    }

    

    /**
     * @param velocity The global velocity of the robot(+X is away from alliance wall, +Y is left facing opposing alliance)
     * @param heading The heading of the robot in radians
     * @return The wheel speeds for the robot
     */
    public DifferentialDriveWheelSpeeds calculate(Vector2d velocity, Rotation2d heading) {
        double desiredGlobalHeading = Math.atan2(velocity.x, velocity.y);
        double localHeading = Util.boundedAngle(heading.getRadians() - desiredGlobalHeading);
        double omega = mTurnController.calculate(localHeading);
        System.out.println("*********************************");
        System.out.println("Desired Global Heading " + desiredGlobalHeading);
        System.out.println("Local Heading " + localHeading);
        System.out.println("Global Heading " + heading.getRadians());
        System.out.println("*********************************");
        ChassisSpeeds localVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.x, velocity.y, omega, heading);
        DifferentialDriveWheelSpeeds speeds = mKinematics.toWheelSpeeds(localVelocity);
        double lVelocity = speeds.leftMetersPerSecond;
        double rVelocity = speeds.rightMetersPerSecond;
        if(Math.abs(lVelocity) > mMaxVelocity || Math.abs(rVelocity) > mMaxVelocity) {
            if(Math.abs(lVelocity) > Math.abs(rVelocity)) {
                lVelocity /= Math.abs(lVelocity / mMaxVelocity);
                rVelocity /= Math.abs(lVelocity / mMaxVelocity);
            } else {
                lVelocity /= Math.abs(rVelocity / mMaxVelocity);
                rVelocity /= Math.abs(rVelocity / mMaxVelocity);
            }
        }
        return new DifferentialDriveWheelSpeeds(lVelocity, rVelocity);
    }
}