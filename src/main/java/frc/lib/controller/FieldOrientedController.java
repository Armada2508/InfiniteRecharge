package frc.lib.controller;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.lib.util.Util;

public class FieldOrientedController {
    private DoubleSupplier mVelocityX;
    private DoubleSupplier mVelocityY;
    private DoubleSupplier mHeading;
    private double mMaxVelocity;
    private PIDController mTurnController;

    
    /**
     * Creates a new FieldOrientedController Object
     */
    public FieldOrientedController(double maxVelocity, PIDController turnController) {
        mMaxVelocity = maxVelocity;
        mTurnController = turnController;
    }

    /**
     * Creates a new FieldOrientedController Object
     * @param velocityX The global X velocity
     * @param velocityY The global Y velocity
     * @param heading The heading of the robot in radians
     * @param maxVelocity The maximum velocity of the robot
     * @param turnController The PID controller used for turning
     */
    public FieldOrientedController(DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier heading, double maxVelocity, PIDController turnController) {
        mVelocityX = velocityX;
        mVelocityY = velocityY;
        mHeading = heading;
        mMaxVelocity = maxVelocity;
        mTurnController = turnController;
    }

    /**
     * @return The wheel speeds for the robot
     */
    public DifferentialDriveWheelSpeeds calculate() {
        if(mVelocityX == null || mVelocityY == null || mHeading == null) {
            return new DifferentialDriveWheelSpeeds(0.0, 0.0);
        }
        return calculate(mVelocityX.getAsDouble(), mVelocityY.getAsDouble(), mHeading.getAsDouble());
    }
    
    /**
     * Resets the controller
     */
    public void reset() {
        mTurnController.reset();
    }

    

    /**
     * @param velocityX The global X velocity of the robot
     * @param velocityY The global Y velocity of the robot
     * @param heading The heading of the robot in radians
     * @return The wheel speeds for the robot
     */
    public DifferentialDriveWheelSpeeds calculate(double velocityX, double velocityY, double heading) {
        System.out.println("*********************************************");
        Vector2d globalVelocityVector = new Vector2d(velocityX, velocityY);
        double desiredGlobalHeading = Math.atan2(-globalVelocityVector.x, globalVelocityVector.y);
        System.out.println("Global Forward: " + velocityY + ", Global Sideways: " + velocityX);
        System.out.println("Desired Global Heading: " + desiredGlobalHeading);
        System.out.println("Current Global Heading: " + heading);
        System.out.println("Current Heading Delta: " + ( heading - desiredGlobalHeading));
        double localHeading = Util.boundedAngle(heading - desiredGlobalHeading, false);
        System.out.println("Local Heading: " + ( heading - desiredGlobalHeading));
        Vector2d localVelocityVector = new Vector2d(Math.sin(localHeading)*globalVelocityVector.magnitude(), Math.cos(localHeading)*globalVelocityVector.magnitude());
        System.out.println("Local Forward: " + localVelocityVector.y + ", Local Sideways: " + localVelocityVector.x);
        System.out.println("Current Local Heading: " + localHeading);
        double turnPower = mTurnController.calculate(localHeading);
        System.out.println("Turn Power: " + localHeading);
        double lPower = localVelocityVector.y - turnPower;
        double rPower = localVelocityVector.y + turnPower;
        if(Math.abs(lPower) > mMaxVelocity || Math.abs(rPower) > mMaxVelocity) {
            if(Math.abs(lPower) > Math.abs(rPower)) {
                lPower /= Math.abs(lPower / mMaxVelocity);
                rPower /= Math.abs(lPower / mMaxVelocity);
            } else {
                lPower /= Math.abs(rPower / mMaxVelocity);
                rPower /= Math.abs(rPower / mMaxVelocity);
            }
        }
        System.out.println("Left Power: " + lPower + ", Right Power: " + rPower);
        System.out.println("*********************************************");
        return new DifferentialDriveWheelSpeeds(lPower, rPower);
    }
}