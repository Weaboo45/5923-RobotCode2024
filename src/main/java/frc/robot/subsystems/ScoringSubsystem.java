package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.math.geometry.Rotation2d;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
    //arm motors
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;

    //arm CANcoder
    private CANcoderConfiguration configs = new CANcoderConfiguration();
    private CANcoder absoluteEncoder;

    //shooter motors
    private CANSparkMax topShooterMotor;
    private CANSparkMax bottomShooterMotor;

    //intake motor
    private CANSparkMax intakeMotor;

    public ScoringSubsystem() {
      absoluteEncoder = new CANcoder(0);
      configAngleEncoder();
    
      //arm motors
      leftArmMotor = new CANSparkMax(Constants.leftArmMotorID, MotorType.kBrushless);
      rightArmMotor = new CANSparkMax(Constants.rightArmMotorID, MotorType.kBrushless);
        
      //shooter motors
      topShooterMotor = new CANSparkMax(Constants.topShooterMotorID, MotorType.kBrushless);
      bottomShooterMotor = new CANSparkMax(Constants.bottomShooterMotorID, MotorType.kBrushless);
        
      //intake motor
      intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushed);

      //motor configs
      configLeftArmMotor();
      configRightArmMotor();
      configBottorShooterMotor();
      configTopShooterMotor();
      configIntakeMotor();
      
      SmartDashboard.putNumber("Left Arm Encoder Val", getAngle());
  }

  private void configAngleEncoder() {
    configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    absoluteEncoder.getConfigurator().apply(configs);
    absoluteEncoder.getPosition().setUpdateFrequency(100);
    absoluteEncoder.getVelocity().setUpdateFrequency(100);
  }

  private void configRightArmMotor() {
    rightArmMotor.restoreFactoryDefaults();
    rightArmMotor.setSmartCurrentLimit(15);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.enableVoltageCompensation(12);
    rightArmMotor.burnFlash();
  }

  private void configLeftArmMotor() {
    leftArmMotor.restoreFactoryDefaults();
    leftArmMotor.setSmartCurrentLimit(15);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.enableVoltageCompensation(12);
    leftArmMotor.burnFlash();
  }

  private void configTopShooterMotor() {
    topShooterMotor.restoreFactoryDefaults();
    topShooterMotor.setSmartCurrentLimit(30);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.enableVoltageCompensation(12);
    topShooterMotor.burnFlash();
  }

  private void configBottorShooterMotor() {
    bottomShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.setSmartCurrentLimit(30);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.enableVoltageCompensation(12);
    bottomShooterMotor.burnFlash();
  }

  private void configIntakeMotor() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(30);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.enableVoltageCompensation(12);
    intakeMotor.burnFlash();
  }

  public double getAngle(){
    return absoluteEncoder.getPosition().getValueAsDouble();
  }

  public void moveArm(double pivotSpeed){
    rightArmMotor.set(pivotSpeed * -2.0);
    leftArmMotor.set(pivotSpeed * 2.0);
  }

  public void intake(double intakeSpeed){
    intakeMotor.set(intakeSpeed * 2);
  }

  public void shooter(double shooterSpeed){
    topShooterMotor.set(shooterSpeed);
    bottomShooterMotor.set(shooterSpeed);
  }
}
