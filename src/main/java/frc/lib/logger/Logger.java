package frc.lib.logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Logger {

    private static ShuffleboardTab logger = Shuffleboard.getTab("Logger");


    public static void logBoolean(BooleanSupplier value, String name) {
        logger.addBoolean(name, value).withWidget(BuiltInWidgets.kBooleanBox);
    }

    public static void logDouble(DoubleSupplier value, String name) {
        logger.addNumber(name, value).withWidget(BuiltInWidgets.kGraph);
    }

    public static void logString(Supplier<String> value, String name) {
        logger.addString(name, value);
    }

    public static void logPDP() {
        logger.add("Power Distribution Panel", new PowerDistributionPanel()).withWidget(BuiltInWidgets.kPowerDistributionPanel);
    }

    public static void logPDP(PowerDistributionPanel powerDistributionPanel) {
        logger.add("Power Distribution Panel", powerDistributionPanel).withWidget(BuiltInWidgets.kPowerDistributionPanel);
    }

    
}