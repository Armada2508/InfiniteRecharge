package frc.lib.logger;

import java.util.Random;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Logger {
    private static ShuffleboardTab logger = Shuffleboard.getTab("Logger");
    private static NetworkTable loggerTable = NetworkTableInstance.getDefault().getTable("Shuffleboard/Logger");
    private static Random mRandom = new Random();

    public static void newNoise() {
        mRandom = new Random();        
    }

    public static double addNoise(double value) {
        return value + (mRandom.nextDouble() / 10000.0);
    }

    public static void log(boolean value, String name) {
        if(loggerTable.getEntry(name).getValue().isValid()) {
            loggerTable.getEntry(name).setBoolean(value);
        } else {
            logger.add(name, value).withWidget(BuiltInWidgets.kBooleanBox);
        }
    }

    public static void log(double value, String name) {
        if(loggerTable.getEntry(name).getValue().isValid()) {
            loggerTable.getEntry(name).setDouble(value);
        } else {
            logger.add(name, value);
        }
    }

    public static void log(String value, String name) {
        if(loggerTable.getEntry(name).getValue().isValid()) {
            loggerTable.getEntry(name).setString(value);
        } else {
            logger.add(name, value);
        }
    }

    public static void logHistory(boolean value, String name) {
        if(loggerTable.getEntry(name).getValue().isValid()) {
            loggerTable.getEntry(name).setBoolean(value);
        } else {
            logger.add(name, value).withWidget(BuiltInWidgets.kGraph);
        }
    }

    public static void logHistory(int value, String name) {
        if(loggerTable.getEntry(name).getValue().isValid()) {
            loggerTable.getEntry(name).setDouble(value);
        } else {
            logger.add(name, value).withWidget(BuiltInWidgets.kGraph);
        }
    }

    public static void logHistory(double value, String name) {
        if(loggerTable.getEntry(name).getValue().isValid()) {
            loggerTable.getEntry(name).setDouble(value);
        } else {
            logger.add(name, value).withWidget(BuiltInWidgets.kGraph);
        }
    }

    public static void logPDP() {
        logger.add("Power Distribution Panel", new PowerDistributionPanel()).withWidget(BuiltInWidgets.kPowerDistributionPanel);
    }

    public static void logPDP(PowerDistributionPanel powerDistributionPanel) {
        logger.add("Power Distribution Panel", powerDistributionPanel).withWidget(BuiltInWidgets.kPowerDistributionPanel);
    }

    
}