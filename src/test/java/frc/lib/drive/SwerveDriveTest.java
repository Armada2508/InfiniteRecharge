package frc.lib.drive;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.Test;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.lib.util.Util;

public class SwerveDriveTest {

    @Test
    public void instantiationTest() {
        Translation2d[] wheelPositions = {
            new Translation2d(1.0, 1.0),
            new Translation2d(-1.0, 1.0),
            new Translation2d(-1.0, -1.0),
            new Translation2d(1.0, -1.0)
        };
        SwerveDrive drive = new SwerveDrive(wheelPositions);
        assertNotNull(drive);
    }

    @Test
    public void translationTest() {
        Translation2d[] wheelPositions = {
            new Translation2d(1.0, 1.0),
            new Translation2d(-1.0, 1.0),
            new Translation2d(-1.0, -1.0),
            new Translation2d(1.0, -1.0)
        };
        SwerveDrive drive = new SwerveDrive(wheelPositions);
        SwerveModuleState[] states;
        for (int i = 0; i < wheelPositions.length; i++) {
            states = drive.calculate(1.0, 0.0, 0.0, 5.0).clone();
            assertEquals(1.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(0.0, 1.0, 0.0, 5.0).clone();
            assertEquals(1.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(-1.0, 0.0, 0.0, 5.0).clone();
            assertEquals(1.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(0.0, -1.0, 0.0, 5.0).clone();
            assertEquals(1.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(1.0, 1.0, 0.0, 5.0).clone();
            assertEquals(new Translation2d(1.0, 1.0).getNorm(), states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(-1.0, 1.0, 0.0, 5.0).clone();
            assertEquals(new Translation2d(1.0, 1.0).getNorm(), states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(5.0, 5.0, 0.0, 5.0).clone();
            assertEquals(5.0, states[i].speedMetersPerSecond, Util.kEpsilon);
        }
        states = drive.calculate(1.0, 0.0, 0.0, 5.0).clone();
        assertEquals(Math.toRadians(0), states[0].angle.getRadians(), Util.kEpsilon);
        states = drive.calculate(0.0, 1.0, 0.0, 5.0).clone();
        assertEquals(Math.toRadians(90), states[0].angle.getRadians(), Util.kEpsilon);
        states = drive.calculate(-1.0, 0.0, 0.0, 5.0).clone();
        assertEquals(Math.toRadians(180), states[0].angle.getRadians(), Util.kEpsilon);
        states = drive.calculate(0.0, -1.0, 0.0, 5.0).clone();
        assertEquals(Math.toRadians(-90), states[0].angle.getRadians(), Util.kEpsilon);
    }

    @Test
    public void triangleTranslationTest() {
        Translation2d[] wheelPositions = {
            new Translation2d(0.0, 1.0),
            new Translation2d(-0.8, -0.2),
            new Translation2d(0.8, -0.2)
        };
        SwerveDrive drive = new SwerveDrive(wheelPositions);
        SwerveModuleState[] states;
        for (int i = 0; i < wheelPositions.length; i++) {
            states = drive.calculate(1.0, 0.0, 0.0, 5.0).clone();
            assertEquals(1.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(0.0, 1.0, 0.0, 5.0).clone();
            assertEquals(1.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(-1.0, 0.0, 0.0, 5.0).clone();
            assertEquals(1.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(0.0, -1.0, 0.0, 5.0).clone();
            assertEquals(1.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(1.0, 1.0, 0.0, 5.0).clone();
            assertEquals(new Translation2d(1.0, 1.0).getNorm(), states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(-1.0, 1.0, 0.0, 5.0).clone();
            assertEquals(new Translation2d(1.0, 1.0).getNorm(), states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(5.0, 5.0, 0.0, 5.0).clone();
            assertEquals(5.0, states[i].speedMetersPerSecond, Util.kEpsilon);
        }
        
        
      }


    @Test
    public void twistTest() {
        Translation2d[] wheelPositions = {
            new Translation2d(1.0, 1.0),
            new Translation2d(-1.0, 1.0),
            new Translation2d(-1.0, -1.0),
            new Translation2d(1.0, -1.0)
        };
        SwerveDrive drive = new SwerveDrive(wheelPositions);
        SwerveModuleState[] states;
        for (int i = 0; i < wheelPositions.length; i++) {
            states = drive.calculate(0.0, 0.0, 1.0, 5.0);
            assertEquals(wheelPositions[i].getNorm(), states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(0.0, 0.0, -1.0, 5.0);
            assertEquals(wheelPositions[i].getNorm(), states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(0.0, 0.0, 5.0, 5.0);
            assertEquals(5.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(0.0, 0.0, -5.0, 5.0);
            assertEquals(5.0, states[i].speedMetersPerSecond, Util.kEpsilon);
        }
    }


    @Test
    public void triangleTwistTest() {
        Translation2d[] wheelPositions = {
            new Translation2d(0.0, 1.0),
            new Translation2d(-0.8, -0.2),
            new Translation2d(0.8, -0.2)
        };
        SwerveDrive drive = new SwerveDrive(wheelPositions);
        SwerveModuleState[] states;
        for (int i = 0; i < wheelPositions.length; i++) {
            states = drive.calculate(0.0, 0.0, 1.0, 5.0);
            assertEquals(wheelPositions[i].getNorm(), states[i].speedMetersPerSecond, Util.kEpsilon);
            states = drive.calculate(0.0, 0.0, -1.0, 5.0);
            assertEquals(wheelPositions[i].getNorm(), states[i].speedMetersPerSecond, Util.kEpsilon);
            if(i == 0) {
                states = drive.calculate(0.0, 0.0, 5.0, 5.0);
                assertEquals(5.0, states[i].speedMetersPerSecond, Util.kEpsilon);
                states = drive.calculate(0.0, 0.0, -5.0, 5.0);
                assertEquals(5.0, states[i].speedMetersPerSecond, Util.kEpsilon);
            }
        }
    }

    @Test
    public void combinedTest() {
        Translation2d[] wheelPositions = {
            new Translation2d(1.0, 1.0),
            new Translation2d(-1.0, 1.0),
            new Translation2d(-1.0, -1.0),
            new Translation2d(1.0, -1.0)
        };
        SwerveDrive drive = new SwerveDrive(wheelPositions);
        SwerveModuleState[] states;
        states = drive.calculate(0.0, 1.0, 1.0, 50.0).clone();
        assertEquals(new Translation2d(1.0, 0.0).getNorm(), states[0].speedMetersPerSecond, Util.kEpsilon);
        assertEquals(new Translation2d(1.0, 2.0).getNorm(), states[1].speedMetersPerSecond, Util.kEpsilon);
        assertEquals(new Translation2d(-1.0, 2.0).getNorm(), states[2].speedMetersPerSecond, Util.kEpsilon);
        assertEquals(new Translation2d(-1.0, 0.0).getNorm(), states[3].speedMetersPerSecond, Util.kEpsilon);
        states = drive.calculate(1.0, 0.0, 1.0, 50.0).clone();
        assertEquals(new Translation2d(2.0, -1.0).getNorm(), states[0].speedMetersPerSecond, Util.kEpsilon);
        assertEquals(new Translation2d(2.0, 1.0).getNorm(), states[1].speedMetersPerSecond, Util.kEpsilon);
        assertEquals(new Translation2d(0.0, 1.0).getNorm(), states[2].speedMetersPerSecond, Util.kEpsilon);
        assertEquals(new Translation2d(0.0, -1.0).getNorm(), states[3].speedMetersPerSecond, Util.kEpsilon);
        states = drive.calculate(0.0, 1.0, -1.0, 50.0).clone();
        assertEquals(new Translation2d(-1.0, 2.0).getNorm(), states[0].speedMetersPerSecond, Util.kEpsilon);
        assertEquals(new Translation2d(-1.0, 0.0).getNorm(), states[1].speedMetersPerSecond, Util.kEpsilon);
        assertEquals(new Translation2d(1.0, 0.0).getNorm(), states[2].speedMetersPerSecond, Util.kEpsilon);
        assertEquals(new Translation2d(1.0, 2.0).getNorm(), states[3].speedMetersPerSecond, Util.kEpsilon);
    
    }

}