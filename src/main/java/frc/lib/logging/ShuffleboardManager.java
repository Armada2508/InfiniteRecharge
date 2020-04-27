package frc.lib.logging;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShuffleboardManager {

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
     * Log a double value on Shuffleboard
     * @param value A supplier for the double value
     * @param name The name to give the value on Shuffleboard
     * @param tab The Shuffleboard tab to add the value to
     */
    public static void logDouble(DoubleSupplier value, String name, String tab) {
        Shuffleboard.getTab(tab).addNumber(name, value);
    }

    /**
     * Log a double value on Shuffleboard with a graph
     * @param value A supplier for the double value
     * @param name The name to give the value on Shuffleboard
     * @param tab The Shuffleboard tab to add the value to
     */
    public static void logDoubleHistory(DoubleSupplier value, String name, String tab) {
        Shuffleboard.getTab(tab).addNumber(name, value).withWidget(BuiltInWidgets.kGraph);
    }

    /**
     * Log a string value on Shuffleboard
     * @param value A supplier for the string value
     * @param name The name to give the value on Shuffleboard
     * @param tab The Shuffleboard tab to add the value to
     */
    public static void logString(Supplier<String> value, String name, String tab) {
        Shuffleboard.getTab(tab).addString(name, value);
    }
    
    /**
     * Log the PDP on Shuffleboard
     * @param tab The Shuffleboard tab to add the PDP to
     */
    public static void logPDP(String tab) {
        Shuffleboard.getTab(tab).add("Power Distribution Panel", new PowerDistributionPanel()).withWidget(BuiltInWidgets.kPowerDistributionPanel);
    }

    /**
     * Log the PDP on Shuffleboard
     * @param powerDistributionPanel The PDP to log
     * @param tab The Shuffleboard tab to add the PDP to
     */
    public static void logPDP(PowerDistributionPanel powerDistributionPanel, String tab) {
        Shuffleboard.getTab(tab).add("Power Distribution Panel", powerDistributionPanel).withWidget(BuiltInWidgets.kPowerDistributionPanel);
    }

    
}