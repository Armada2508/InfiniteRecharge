package frc.lib.logging;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class ShuffleboardManager {

    private static ShuffleboardTab logger = Shuffleboard.getTab("Logger");

    /**
     * Log a boolean value on Shuffleboard
     * @param value A supplier for the boolean value
     * @param name The name to give the value on Shuffleboard
     * @param tab The Shuffleboard tab to add the value to
     */
    public static void logBoolean(BooleanSupplier value, String name, String tab) {
        Shuffleboard.getTab(tab).addBoolean(name, value).withWidget(BuiltInWidgets.kBooleanBox);
    }

    /**
     * Log a boolean value on Shuffleboard
     * @param value A supplier for the boolean value
     * @param name The name to give the value on Shuffleboard
     */
    public static void logBoolean(BooleanSupplier value, String name) {
        logger.addBoolean(name, value).withWidget(BuiltInWidgets.kBooleanBox);
    }

    /**
     * Log a double value on Shuffleboard
     * @param value A supplier for the double value
     * @param name The name to give the value on Shuffleboard
     */
    public static void logDouble(DoubleSupplier value, String name) {
        logger.addNumber(name, value).withWidget(BuiltInWidgets.kGraph);
    }

    /**
     * Log a double value on Shuffleboard
     * @param value A supplier for the double value
     * @param name The name to give the value on Shuffleboard
     * @param tab The Shuffleboard tab to add the value to
     */
    public static void logDouble(DoubleSupplier value, String name, String tab) {
        logger.addNumber(name, value).withWidget(BuiltInWidgets.kGraph);
    }

    /**
     * Log a string value on Shuffleboard
     * @param value A supplier for the string value
     * @param name The name to give the value on Shuffleboard
     */
    public static void logString(Supplier<String> value, String name) {
        logger.addString(name, value);
    }

    /**
     * Log a string value on Shuffleboard
     * @param value A supplier for the string value
     * @param name The name to give the value on Shuffleboard
     * @param tab The Shuffleboard tab to add the value to
     */
    public static void logString(Supplier<String> value, String name, String tab) {
        logger.addString(name, value);
    }

    /**
     * Log the PDP on Shuffleboard
     */
    public static void logPDP() {
        logger.add("Power Distribution Panel", new PowerDistributionPanel()).withWidget(BuiltInWidgets.kPowerDistributionPanel);
    }

    /**
     * Log the PDP on Shuffleboard
     * @param powerDistributionPanel The PDP to log
     */
    public static void logPDP(PowerDistributionPanel powerDistributionPanel) {
        logger.add("Power Distribution Panel", powerDistributionPanel).withWidget(BuiltInWidgets.kPowerDistributionPanel);
    }

    
}