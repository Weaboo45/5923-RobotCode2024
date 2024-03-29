/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.manual.JoyStickCommands.*;
import frc.robot.commands.manual.ControllerCommands.*;
import frc.robot.commands.automatic.*;
import frc.robot.commands.autonomous.ScoreAmp;
import frc.robot.commands.autonomous.SimpleAutonomous;
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
  private final SendableChooser<Command> m_chooser;
    private final ScoreAmp scoreAmp = new ScoreAmp(scoreSub);

  public RobotContainer() {
    m_chooser = AutoBuilder.buildAutoChooser();

    configureInitialDefaultCommands();
    configureBindings();
    configureShuffleboardData();
    configureSmartDashboard();

    NamedCommands.registerCommand("Intake", intakeComand);
    NamedCommands.registerCommand("Shoot", shootComand);

    NamedCommands.registerCommand("ScoreAmp", scoreAmp);

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  // The robot's subsystems and commands are defined here...
  /// SHUFFLEBOARD TAB ///
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Competition Robot");

  /// SUBSYSTEMS ///
  public static final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  public static final ScoringSubsystem scoreSub = new ScoringSubsystem();
  public static final ArmSubsystem armSub = new ArmSubsystem();

  /// OI DEVICES / HARDWARE ///
  private final XboxController xbox = new XboxController(0);
  private final PS4Controller ps4 = new PS4Controller(0);
  private final Joystick stick = new Joystick(1);
  private static final AHRS ahrs = new AHRS(Port.kMXP);

  CommandXboxController commandXbox = new CommandXboxController(0);


  /// COMMANDS ///
  // Auto Commands
  private final SimpleAutonomous simpleAuto = new SimpleAutonomous(scoreSub, drivetrain);

  private final AutoIntake intakeComand = new AutoIntake(scoreSub);
  private final AutoShoot shootComand = new AutoShoot(scoreSub);

  // Xbox controls
  private final DriveSwerve drivetrainXbox = new DriveSwerve(drivetrain, () -> -xbox.getLeftY(), ()-> xbox.getLeftX(), ()-> -xbox.getRightX(),
   () -> xbox.getRightBumper(), ()-> xbox.getLeftBumper()); //RB toggles field orintation || LB resets heading

  // Playstation Controls
  private final DriveSwerve drivePlaystation = new DriveSwerve(drivetrain, () -> -ps4.getLeftY(), () -> ps4.getLeftX(),() -> -ps4.getRightX(),
   () -> ps4.getR1Button(), () -> ps4.getL1Button()); //R1 toggles field orintation || L1 button resets heading

  // Joystick Controls
  private final DriveJoystickSwerve driveJoystick = new DriveJoystickSwerve(drivetrain, () -> stick.getY(), () -> stick.getX(), () -> stick.getTwist(),
   () -> stick.getRawButton(7), () -> stick.getRawButton(8), () -> stick.getThrottle());
  
  // Arm Command
  private final ArmCommand ps4Arm = new ArmCommand(scoreSub, () -> ps4.getR2Axis(), () -> ps4.getL2Axis(),
  () -> ps4.getSquareButton(), () -> ps4.getTriangleButton(), () -> ps4.getCrossButton());

  private final JoystickArmCommand joystickArm = new JoystickArmCommand(scoreSub, () -> stick.getRawButton(6), () -> stick.getRawButton(4), () -> stick.getTrigger(),
  () -> stick.getRawButton(3), () -> stick.getRawButton(2));
  /// SHUFFLEBOARD METHODS ///
  /**
   * Use this command to define {@link Shuffleboard} buttons using a
   * {@link ShuffleboardTab} and its add() function. You can put already defined
   * Commands,
   */
  private void configureShuffleboardData() {
    Shuffleboard.selectTab(m_tab.getTitle());

    ShuffleboardLayout encoders = m_tab.getLayout("Encoders", BuiltInLayouts.kGrid);
    encoders.add("Encoder Reset", new InstantCommand(()-> drivetrain.resetToAbsolute()));

    ShuffleboardLayout drivingStyleLayout = m_tab.getLayout("driving styles", BuiltInLayouts.kList)
    .withPosition(0, 0).withSize(2, 2)
    .withProperties(Map.of("label position", "BOTTOM"));

    drivingStyleLayout.add("Xbox Drive",
    new InstantCommand(() -> drivetrain.setDefaultCommand(drivetrainXbox), drivetrain));

    drivingStyleLayout.add("PS5 Drive",
    new InstantCommand(() -> drivetrain.setDefaultCommand(drivePlaystation), drivetrain));

    drivingStyleLayout.add("PS5 Arm Controll",
    new InstantCommand(() -> scoreSub.setDefaultCommand(ps4Arm), scoreSub));

    drivingStyleLayout.add("Joystick Drive",
    new InstantCommand(() -> drivetrain.setDefaultCommand(driveJoystick), drivetrain));

    drivingStyleLayout.add("Joystick Arm Controll",
    new InstantCommand(() -> scoreSub.setDefaultCommand(joystickArm), scoreSub));

 
    ShuffleboardLayout gyroSensor = m_tab.getLayout("NavX", BuiltInLayouts.kGrid)
    .withPosition(2, 0).withSize(1, 3)
    .withProperties(Map.of("label position", "BOTTOM"));

    gyroSensor.addNumber("Gyro", ()-> ahrs.getYaw()).withWidget(BuiltInWidgets.kGyro);

    gyroSensor.add("Reset",
    new InstantCommand(()-> drivetrain.zeroHeading()));

    ShuffleboardLayout controllerLayout = m_tab.getLayout("Controller Vals", BuiltInLayouts.kGrid)
    .withPosition(4, 0).withSize(2, 6)
    .withProperties(Map.of("label position", "BOTTOM"));
    controllerLayout.addNumber("left y", () -> -ps4.getLeftY())
    .withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left x", () -> ps4.getLeftX())
    .withPosition(0, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left trigger", () -> ps4.getL2Axis())
    .withPosition(0, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right y", () -> -ps4.getRightY())
    .withPosition(2, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right x", () -> ps4.getRightX())
    .withPosition(2, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right trigger", () -> ps4.getR2Axis())
    .withPosition(2, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
  }

  private void configureSmartDashboard(){
    //match Auto
    m_chooser.setDefaultOption("Simple Auton", simpleAuto);
    m_chooser.addOption("Amp Auto", new PathPlannerAuto("AmpAuto"));
    m_chooser.addOption("Right Side Path", new PathPlannerAuto("Right Side Path"));

    //test Autos
    m_chooser.addOption("Square", new PathPlannerAuto("SquareAuto"));
    m_chooser.addOption("Test Auto", new PathPlannerAuto("Test Auto"));
    m_chooser.addOption("Intake and shooter test", new PathPlannerAuto("IntakeShoot"));
    SmartDashboard.putData(m_chooser);
  }

  /**   
   * Use this method to define the default commands of subsystems. 
   * Default commands are ran whenever no other commands are using a specific subsystem.
   */
  private void configureInitialDefaultCommands() {
    drivetrain.setDefaultCommand(drivetrainXbox);
    scoreSub.setDefaultCommand(joystickArm);
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    SmartDashboard.putData("Intake Auto", simpleAuto);

    //button comands for arm
    commandXbox.a().onTrue
    (Commands.runOnce(
      ()-> {
        armSub.setGoal(2);
        armSub.enable();
      }, armSub));

      commandXbox.b().onTrue
    (Commands.runOnce(
      ()-> {
        armSub.setGoal(.5);
        armSub.enable();
      }, armSub));

      commandXbox.y().onTrue(Commands.runOnce(armSub::disable));
  }

  /**
   * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be called on robot
   * disable to prevent integral windup.
   */
  public void disablePIDSubsystems() {
    armSub.disable();
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Executes the autonomous command chosen in smart dashboard

    // Load the path you want to follow using its name in the GUI
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        //return AutoBuilder.followPath(path);
    return m_chooser.getSelected();
  }

  public void displayValues() {
  SmartDashboard.putData(drivetrain);
  SmartDashboard.putData(m_chooser);
  }
}
