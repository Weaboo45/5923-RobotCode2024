package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final PWMSparkMax rightArmMotor = new PWMSparkMax(ArmConstants.rightArmMotorID);
  private final PWMSparkMax leftArmMotor = new PWMSparkMax(ArmConstants.leftArmMotorID);

  private final Encoder absoluteEncoder = new Encoder(2, 3);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            ArmConstants.kI,
            ArmConstants.kD,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    absoluteEncoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);
    // Start arm at rest in neutral position
    setGoal(ArmConstants.angleOffsetDegree);

    leftArmMotor.setInverted(true);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    rightArmMotor.setVoltage(output + feedforward);
    leftArmMotor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return absoluteEncoder.getDistance() + ArmConstants.angleOffsetDegree;
  }
}
