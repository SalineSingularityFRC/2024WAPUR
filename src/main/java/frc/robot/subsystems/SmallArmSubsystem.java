package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SmallArmSubsystem extends SubsystemBase {

  // Talon Setup
  private TalonFX smallArmMotor;

  private PositionVoltage positionTargetPreset = new PositionVoltage(0).withSlot(0).withEnableFOC(true);

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0.2);
  private TalonFXConfiguration talonFXConfigsPreset = new TalonFXConfiguration();
  private MotionMagicConfigs motionMagicConfigsPresets;
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
  

  private final double manualBigP = 1;
  private final double manualBigI = 0;
  private final double manualBigD = 0;
  private final double manualBigS = 0.03;

  // OLD PID CONSTANTS
  private final double slot0P = 0.4;
  private final double slot0I = 0;
  private final double slot0D = 0.1;
  private final double slot0S = 0.0;

  public double smallArmMotorPosition;

  public SmallArmSubsystem() {


    smallArmMotor = new TalonFX(Constants.CanId.Arm.Motor.SMALL, Constants.Canbus.DEFAULT);

    smallArmMotor.getConfigurator().apply(new TalonFXConfiguration());

    HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs();
    limitSwitchConfigs.ReverseLimitAutosetPositionEnable = true;
    limitSwitchConfigs.ReverseLimitAutosetPositionValue = 0;
    limitSwitchConfigs.ForwardLimitAutosetPositionEnable = false;

    Slot0Configs slot0ConfigsBig = new Slot0Configs();
    slot0ConfigsBig.kP = slot0P;
    slot0ConfigsBig.kI = slot0I;
    slot0ConfigsBig.kD = slot0D;
    slot0ConfigsBig.kS = slot0S;

    Slot1Configs slot1ConfigsBig = new Slot1Configs();
    slot1ConfigsBig.kP = manualBigP;
    slot1ConfigsBig.kI = manualBigI;
    slot1ConfigsBig.kD = manualBigD;
    slot1ConfigsBig.kS = manualBigS;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = 30;

    smallArmMotor.getConfigurator().apply(slot0ConfigsBig);
    smallArmMotor.getConfigurator().apply(slot1ConfigsBig);
    smallArmMotor.getConfigurator().apply(currentLimitsConfigs);

    motionMagicConfigsPresets = talonFXConfigsPreset.MotionMagic;
    motionMagicConfigsPresets.MotionMagicCruiseVelocity = 40 / 30;
    motionMagicConfigsPresets.MotionMagicAcceleration = 100 / 80;
    motionMagicConfigsPresets.MotionMagicJerk = 900 / 60;

    smallArmMotor.getConfigurator().apply(motionMagicConfigsPresets);

    // Global Position Value
    smallArmMotorPosition = smallArmMotor.getPosition().getValue();

    setBrakeMode();
  }

  public void setArmSpeed(double smallArmspeed) {
    smallArmMotor.setControl(dutyCycleOut.withOutput(smallArmspeed).withEnableFOC(true));
    smallArmMotorPosition = smallArmMotor.getPosition().getValue();
  }

  public void setPosition(double smallArmAngle) {
    smallArmMotor.setControl(
        positionTargetPreset.withPosition(smallArmAngle).withFeedForward(0.1).withSlot(0));
    smallArmMotorPosition = smallArmAngle;
  }

  public void setBrakeMode() {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    smallArmMotor.getConfigurator().apply(motorOutputConfigs);
  }

  public void periodic() {

  }

  public Command stopArm() {
    return run(
        () -> {
          maintainSmallArmPosition();
        });
  }

  public Command moveArmForward() {
    return run(
        () -> {
          setArmSpeed(Constants.Speed.ARMDUTYCYCLEUPSMALL);
        });
  }

  public Command moveSmallArmForward() {
    return run(
        () -> {
          setArmSpeed(Constants.Speed.ARMDUTYCYCLEUPSMALL);
        });
  }  

  public Command moveArmBackwards() {
    return run(
        () -> {
          setArmSpeed(-Constants.Speed.ARMDUTYCYCLEDOWNSMALL);
        });
  }

  public Command moveSmallArmBackwards() {
    return run(
        () -> {
          setArmSpeed(-Constants.Speed.ARMDUTYCYCLEDOWNSMALL);
        });
  }

  // public Command shootTarget() {
  //   return new FunctionalCommand(
  //       () -> {

  //       },
  //       () -> {
  //         setPosition(Constants.Position.MainArm.Speaker.FEET3,0);
  //       },
  //       (_unused) -> {

  //       },
  //       () -> {
  //         return Math.abs(Constants.Position.MainArm.Speaker.FEET3 - baseArmMotor1.getPosition().getValueAsDouble()) < 0.5;
  //       },
  //       this);
  // }

  // public Command autonShootTarget(double pos) {
  //   return new FunctionalCommand(
  //       () -> {

  //       },
  //       () -> {
  //         setPosition(pos,0);
  //       },
  //       (_unused) -> {

  //       },
  //       () -> {

  //         return Math.abs(pos - baseArmMotor1.getPosition().getValueAsDouble()) < 1;
  //       },
  //       this);
  // }

  // public Command limelightShootTarget(Limelight lime) {
  //   return new FunctionalCommand(
  //       () -> {
        
  //       },
  //       () -> {
  //         double distance = lime.getDistanceToTagInFeet();
  //         double toDriveDistance = 0;

  //         if (distance > 6) {
  //           toDriveDistance = 6;
  //         }
  //         else {
  //           toDriveDistance = distance;
  //         }
        
  //         double shootingPos = lime.getArmPositionFromDistance(toDriveDistance);
  //         SmartDashboard.putNumber("shootingPosLimelight", shootingPos);

  //         baseArmMotor1.setControl(
  //         positionTargetPreset.withPosition(shootingPos).withFeedForward(0.1).withSlot(0));
  //         baseArmMotorPosition = shootingPos;
  //       },
  //       (_unused) -> {

  //       },
  //       () -> {
  //         double distance = lime.getDistanceToTagInFeet();
  //         double toDriveDistance = 0;

  //         if (distance > 6) {
  //           toDriveDistance = 6;
  //         }
  //         else {
  //           toDriveDistance = distance;
  //         }
        
  //         double shootingPos = lime.getArmPositionFromDistance(toDriveDistance);
  //         return Math.abs(shootingPos- baseArmMotor1.getPosition().getValueAsDouble()) < 1;
  //       },
  //       this);
  // }

  public Command pickupTarget() {
    return runOnce(() -> {
      setPosition(Constants.Position.MainArm.PICKUP);
    });
  }

  public Command maintainSmallArm() {
    return run(() -> {
      maintainSmallArmPosition();
    });
  }

  public void maintainSmallArmPosition() {
    // System.out.println("MAINTAIN");
    // baseArmMotor1.getConfigurator().apply(motionMagicConfigsPresets);
    smallArmMotor.setControl(
      positionTargetPreset.withPosition(smallArmMotorPosition).withFeedForward(0.03 * 12).withSlot(0));
  }

  public double getPosition() {
    return (smallArmMotor.getPosition().getValue() * 2 * Math.PI) / Constants.MotorGearRatio.ARM;
  }
}
