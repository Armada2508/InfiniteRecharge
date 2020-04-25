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
     * @return The wheel speeds fo each side of the robot
     */
    public DifferentialDriveWheelSpeeds calculate() {
        Vector2d globalDirectionVector = new Vector2d(mVelocityX.getAsDouble(), mVelocityY.getAsDouble());
        if(globalDirectionVector.magnitude() > 1.0) {
            globalDirectionVector.x /= globalDirectionVector.magnitude();
            globalDirectionVector.y /= globalDirectionVector.magnitude();
        }
        double globalHeading = Math.atan2(globalDirectionVector.x, globalDirectionVector.y);
        double localDirectionHeading = Util.boundedAngle(globalHeading - mHeading.getAsDouble(), false);
        Vector2d localVelocityVector = new Vector2d(mMaxVelocity*Math.sin(localDirectionHeading)*globalDirectionVector.magnitude(), mMaxVelocity*Math.sin(localDirectionHeading)*globalDirectionVector.magnitude());
        double turnPower = mTurnController.calculate(localDirectionHeading);
        return new DifferentialDriveWheelSpeeds(localVelocityVector.y + turnPower, localVelocityVector.y - turnPower);
    }
}