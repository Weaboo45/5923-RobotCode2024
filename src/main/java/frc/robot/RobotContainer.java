/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;


//import java.util.HashMap;
import java.util.Map;
//import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

/*
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
*/

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.autonomous.SimpleAutonomous;
import frc.robot.commands.manual.DriveJoystickSwerve;
import frc.robot.commands.manual.DriveSwerve;

import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  public RobotContainer() {
    configureInitialDefaultCommands();
    configureButtonBindings();
    configureShuffleboardData();
    configureSmartDashboard();
  }

  // The robot's subsystems and commands are defined here...
  /// SHUFFLEBOARD TAB ///
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Competition Robot");
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  
  /* 
  private final SendableChooser<Optional<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();
  private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    drivetrain::getPose,
    drivetrain::resetOdometry,
      Constants.DRIVE_KIN, // SwerveDriveKinematics
      new PIDConstants(1, 0, 0),
      new PIDConstants(.8, 0, 0),
            drivetrain::setModuleStates,
      eventMap,
      true,
      drivetrain);

  private static Map<String, Command> eventMap = new HashMap<>();


  PathPlannerTrajectory DoNothing = PathPlanner.loadPath("DoNothing",0,0);
*/

  /// SUBSYSTEMS ///
  public static final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  /// OI DEVICES / HARDWARE ///
  private final XboxController xbox = new XboxController(0);
  private final PS4Controller ps4 = new PS4Controller(1);
  private final Joystick stick = new Joystick(1);
  private static final AHRS ahrs = new AHRS(Port.kMXP);


  /// COMMANDS ///
  // Autonomous
  private final SimpleAutonomous simpleAuto = new SimpleAutonomous(drivetrain, ahrs);

  // Xbox controls
  private final DriveSwerve drivetrainXbox = new DriveSwerve(drivetrain, () -> -xbox.getLeftY(), ()-> xbox.getLeftX(), ()-> -xbox.getRightX(),
   () -> xbox.getRightBumper(), ()-> xbox.getLeftBumper(), ()-> xbox.getAButton()); //RB toggles field orintation || LB toggles speed || A button resets heading

  // Playstation Controls
  private final DriveSwerve drivePlaystation = new DriveSwerve(drivetrain, () -> -ps4.getLeftY(), () -> ps4.getLeftX(),() -> -ps4.getRightX(),
   () -> ps4.getR1Button(), () -> ps4.getL1Button(), () -> ps4.getCrossButton()); //R1 toggles field orintation || L1 toggles speed || X button resets heading

  // Joystick Controls
  private final DriveJoystickSwerve driveJoystick = new DriveJoystickSwerve(drivetrain, () -> stick.getY(), () -> stick.getX(), () -> stick.getTwist(),
   () -> stick.getRawButton(7), () -> stick.getRawButton(8), () -> stick.getThrottle());
  
  /// SHUFFLEBOARD METHODS ///
  /**
   * Use this command to define {@link Shuffleboard} buttons using a
   * {@link ShuffleboardTab} and its add() function. You can put already defined
   * Commands,
   */
  private void configureShuffleboardData() {
    Shuffleboard.selectTab(m_tab.getTitle());
    
    m_chooser.setDefaultOption("Basic Autonomous Sequence", simpleAuto);
    //m_chooser.addOption("Automatic autonomous", pidAuto);

    ShuffleboardLayout encoders = m_tab.getLayout("Encoders", BuiltInLayouts.kGrid);
    encoders.add("Encoder Reset", new InstantCommand(()-> drivetrain.resetToAbsolute()));

    ShuffleboardLayout drivingStyleLayout = m_tab.getLayout("driving styles", BuiltInLayouts.kList)
    .withPosition(0, 0).withSize(2, 2)
    .withProperties(Map.of("label position", "BOTTOM"));

    drivingStyleLayout.add("Xbox Drive",
    new InstantCommand(() -> drivetrain.setDefaultCommand(drivetrainXbox), drivetrain));

    drivingStyleLayout.add("PS4 Drive",
    new InstantCommand(() -> drivetrain.setDefaultCommand(drivePlaystation), drivetrain));

    drivingStyleLayout.add("Joystick Drive",
    new InstantCommand(() -> drivetrain.setDefaultCommand(driveJoystick), drivetrain));

 
    ShuffleboardLayout gyroSensor = m_tab.getLayout("NavX", BuiltInLayouts.kGrid)
    .withPosition(2, 0).withSize(1, 3)
    .withProperties(Map.of("label position", "BOTTOM"));

    gyroSensor.addNumber("Gyro", ()-> ahrs.getYaw()).withWidget(BuiltInWidgets.kGyro);

    gyroSensor.add("Reset",
    new InstantCommand(()-> drivetrain.zeroHeading()));

    ShuffleboardLayout controllerLayout = m_tab.getLayout("Controller Vals", BuiltInLayouts.kGrid)
    .withPosition(4, 0).withSize(2, 6)
    .withProperties(Map.of("label position", "BOTTOM"));
    controllerLayout.addNumber("left y", () -> -xbox.getLeftY())
    .withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left x", () -> xbox.getLeftX())
    .withPosition(0, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left trigger", () -> xbox.getLeftTriggerAxis())
    .withPosition(0, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right y", () -> -xbox.getRightY())
    .withPosition(2, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right x", () -> xbox.getRightX())
    .withPosition(2, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right trigger", () -> xbox.getRightTriggerAxis())
    .withPosition(2, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);

    m_tab.add("Auto Chooser", m_chooser)
    .withPosition(0, 6).withSize(5, 2)
    .withWidget(BuiltInWidgets.kSplitButtonChooser);   
  }

  private void configureSmartDashboard(){
    //autoChooser.addOption("test", Optional.empty());

    //SmartDashboard.putData(autoChooser);
  }

  /**   
   * Use this method to define the default commands of subsystems. 
   * Default commands are ran whenever no other commands are using a specific subsystem.
   */
  private void configureInitialDefaultCommands() {
    drivetrain.setDefaultCommand(driveJoystick);
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   /*
  public Command getAutonomousCommand() {
    // Executes the autonomous command chosen in smart dashboard
    Optional<PathPlannerTrajectory> choice = autoChooser.getSelected();
    if (choice.isEmpty()) {
        return null;
    }
    return new ParallelCommandGroup(
            new InstantCommand(
                    () -> drivetrain.getField().getObject("Field").setTrajectory(
                        choice.get())),
            autoBuilder.fullAuto(choice.get()));
  }
  */
  

  public void displayValues() {
  SmartDashboard.putData(drivetrain);
  }
}