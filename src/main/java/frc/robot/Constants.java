/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.SwerveModuleConstants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int rightArmMotorID = 10;
  public static final int leftArmMotorID = 11;

  public static final int topShooterMotorID = 12;
  public static final int bottomShooterMotorID = 13;

  public static final int intakeMotorID = 14;

    public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); //1.413
      public static final boolean driveMotorInverted = true;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); //141.328
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); //142.466
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); //-126.211
      public static final boolean driveMotorInverted = true;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    // Amp limits
    public static int PEAK_LIMIT = 40;
    public static int ENABLE_LIMIT = 30;

    // MEASUREMENTS
        // Drivetrain measurements
        public static double CENTER_TO_WHEEL_X = Units.inchesToMeters(28/2); // Length
        public static double CENTER_TO_WHEEL_Y = Units.inchesToMeters(28/2); // width
        public static double WHEEL_DIAMETER = Units.inchesToMeters(4);

        //Swerve Kinematics
        public static SwerveDriveKinematics DRIVE_KIN = new SwerveDriveKinematics(
            new Translation2d(-CENTER_TO_WHEEL_X, CENTER_TO_WHEEL_Y), //mod 0
            new Translation2d(-CENTER_TO_WHEEL_X, -CENTER_TO_WHEEL_Y), //mod 1
            new Translation2d(CENTER_TO_WHEEL_X, -CENTER_TO_WHEEL_Y), // mod 2
            new Translation2d(CENTER_TO_WHEEL_X, CENTER_TO_WHEEL_Y)); // mod 3


    // PID CONSTANTS
        // Drivetrain
        public static double ROTATE_KP = 0.01; //4.167 * .001
        public static double ROTATE_KI = 0.00; //4 * .001
        public static double ROTATE_KD = 0.0005; //0 * .001

    // Drivetrain deadbands
    public static double ROTATION_DEADBAND = .25;   //.25
    public static double STRAFING_DEADBAND = .25;  //.75
    public static double SPEED_DEADBAND = .25; //.3

    //Drivetrain maxes
    public static double DRIVETRAIN_MAX_SPEED = 4;
    public static double DRIVETRAIN_MAX_TURN_SPEED = 2;

    //Drive motor Conversion Factors
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;

    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 360 / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;

    //Arm motor Conversion Factors
    public static final double ARM_MOTOR_GEAR_RATIO = 0.1875;
    public static final double ARM_DIAMETER = Units.inchesToMeters(35); //inches

    public static final double ARM_MOTOR_PCONVERSION = ARM_DIAMETER * Math.PI / ARM_MOTOR_GEAR_RATIO;
    public static final double ARM_MOTOR_VCONVERSION = ARM_MOTOR_PCONVERSION / 60.0;

    // Autonomous drivetrain PID
    public static double AUTON_KP = 0;
    public static double AUTON_KI = 0;
    public static double AUTON_KD = 0;
    public static double AUTON_DISTANCE_SETPOINT = 0; // feet 3

    //Odometry
    public static final boolean invertGyro = false;


    public static final double driveKS = 0.1;
    public static final double driveKV = 2.3;
    public static final double driveKA = 0.3;

    public static final Translation2d MODULE_OFFSET = new Translation2d(CENTER_TO_WHEEL_X, CENTER_TO_WHEEL_Y);

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(10.0, 0, 0), // Translation constants 
      new PIDConstants(10.0, 0, 0), // Rotation constants 
      4.0, 
      MODULE_OFFSET.getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig());


      
    public static final class ArmConstants {

    //Arm motor Conversion Factors
    public static final double ARM_MOTOR_GEAR_RATIO = 0.1875;
    public static final double ARM_DIAMETER = Units.inchesToMeters(35);

    public static final double ARM_MOTOR_PCONVERSION = ARM_DIAMETER * Math.PI / ARM_MOTOR_GEAR_RATIO;
    public static final double ARM_MOTOR_VCONVERSION = ARM_MOTOR_PCONVERSION / 60;

    public static final int rightArmMotorID = 10;
    public static final int leftArmMotorID = 11;

    //filler vals
    public static final int [] kEncoderPorts = new int[] {0, 1};

    public static final double kP = 1;

    public static final int kEncoderPPR = 1025; // 975.6  ||try both
    public static final double kEncoderDistancePerPulse = ARM_MOTOR_PCONVERSION / kEncoderPPR;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 2;
    public static final double kMaxAccelerationRadPerSecSquared = 3;

    public static final double kArmOffsetRads = 0.5;
    }
}