package frc.lib.controller;

/**
 * Based on https://www.chiefdelphi.com/t/paper-take-back-half-shooter-wheel-speed-control/121640
 */
public class TakeBackHalfController {

    public double mKI;
    public double mSetpoint;
    public double mError;
    public double mLastControlEffort;
    public double mH0;

    /**
     * Creates a new TakeBackHalfController
     * @param kI The integral gain
     * @param setpoint The setpoint of the controller
     */
    public TakeBackHalfController(double kI, double setpoint) {
        mKI = kI;
        mSetpoint = setpoint;
        mLastControlEffort = 0.0;
    }

    /**
     * Sets the setpoint of the controller
     * @param setpoint
     */
    public void setSetpoint(double setpoint) {
        mSetpoint = setpoint;
    }

    /**
     * Calculates the control effort based on the setpoint and input
     * @param input
     */
    public double calculate(double input) {
        double error = mSetpoint - input;
        double controlEffort = mLastControlEffort + error * mKI;
        if(Math.signum(error) != Math.signum(mError)) {
            controlEffort = (mH0 + controlEffort) / 2.0;
            mH0 = controlEffort;
        }
        mError = error;
        mLastControlEffort = controlEffort;
        return controlEffort;
    }
}