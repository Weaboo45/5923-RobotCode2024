package frc.robot.subsystems;

import frc.robot.Constants;
//import frc.robot.subsystems.SwerveModules;

import java.text.DecimalFormat;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveDrivetrain extends SubsystemBase {
  private SwerveModules[] swerveModules;

  private AHRS gyro = new AHRS();
  private Float rates[] = new Float[3];
  private SwerveDrivePoseEstimator poseEstimator;
  private Field2d field;

  /*
  private static SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  public static SwerveDrivetrain getInstance(){
    return drivetrain;
  }
  */

  public SwerveDrivetrain() {

    swerveModules = new SwerveModules[] {
      new SwerveModules(0, Constants.Mod0.constants),
      new SwerveModules(1, Constants.Mod1.constants),
      new SwerveModules(2, Constants.Mod2.constants),
      new SwerveModules(3, Constants.Mod3.constants)
  };

    // Configure AutoBuilder
    //AutoBuilder.configureCustom(null, this::getPose, this::resetOdometry);
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      ()-> Constants.DRIVE_KIN.toChassisSpeeds(getModuleStates()), 
      this::driveRobotRelative, 
      Constants.pathFollowerConfig,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this);

     // Set up custom logging to add the current path to a field 2d widget
     //PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

     //SmartDashboard.putData("Field", field);


  poseEstimator = new SwerveDrivePoseEstimator(Constants.DRIVE_KIN, getYaw(), getPositions(),
        new Pose2d());

    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();

    field = new Field2d();
    SmartDashboard.putData(field);
  }

  @AutoLogOutput
  public void swerveDrive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){ //Drive with rotational speed control w/ joystick  
    SwerveModuleState[] swerveModuleStates = Constants.DRIVE_KIN.toSwerveModuleStates(
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), rotation, getYaw())
          : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DRIVETRAIN_MAX_SPEED);

    for (SwerveModules mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public Field2d getField() {
    return field;
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.DRIVE_KIN.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  @AutoLogOutput
  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DRIVETRAIN_MAX_SPEED);

    for (SwerveModules mod : swerveModules) {
      mod.setDesiredState(moduleStates[mod.moduleNumber], false);
    }
  }

  @AutoLogOutput
  public void setModuleRotation(Rotation2d rotation) {
    for (SwerveModules mod : swerveModules) {
      mod.setDesiredState(new SwerveModuleState(0, rotation), false);
    }
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose){
    poseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModules mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  } 

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModules mod : swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroHeading(){
    gyro.zeroYaw();
  }

  public Rotation2d getYaw() {
    return (Constants.invertGyro)
        ? Rotation2d.fromDegrees(360 - (gyro.getYaw()% 360))
        : Rotation2d.fromDegrees(gyro.getYaw()% 360);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //poseEstimator.update(getHeadingRotation2d(), getPositions());

    Rotation2d yawValue = getYaw();
    double rawYawValue = gyro.getAngle();

    poseEstimator.update(yawValue, getPositions());

    rates[0] = gyro.getRawGyroX();
    rates[1] = gyro.getRawGyroY();
    rates[2] = gyro.getRawGyroZ();

    field.setRobotPose(getPose());
  
    SmartDashboard.putNumber("Robot Angle", rawYawValue);
    SmartDashboard.putString("Pose", getPose().toString());
    SmartDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((rates[2] / 180)) + "pi rad/s");

    //Pose2d poseA = getPose();
    Logger.recordOutput("Drivetrain/Robot Angle", getYaw().getRadians());
    Logger.recordOutput("Drivetrain/Pose", getPose());
    Logger.recordOutput("Drivetrain/Angular Speed", rates[2] / 180);
    Logger.recordOutput("Drivetrain/Module States", getModuleStates());
  }

  public void resetToAbsolute() {
    for (SwerveModules mod : swerveModules) {
      mod.resetToAbsolute();
    }
  }
}