package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{
  private TalonFX rightMotor;
  private TalonFX leftMotor;
  private DutyCycleEncoder pivotEncoder;

  public Pivot(){
    rightMotor = new TalonFX(PivotConstants.rightPivotID);
    leftMotor = new TalonFX(PivotConstants.leftPivotID);
    pivotEncoder = new DutyCycleEncoder(PivotConstants.pivotEncoderID);

    //Left Config Setup
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();

		leftConfig.CurrentLimits.StatorCurrentLimitEnable =  true;
		leftConfig.CurrentLimits.SupplyCurrentLimitEnable =  true;
		leftConfig.CurrentLimits.StatorCurrentLimit =  80;
		leftConfig.CurrentLimits.SupplyCurrentLimit =  30;	
		leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		leftConfig.Voltage.PeakForwardVoltage = 12;
		leftConfig.Voltage.PeakReverseVoltage = -12;

    //Right Config Setup
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    rightConfig.CurrentLimits.StatorCurrentLimitEnable =  true;
		rightConfig.CurrentLimits.SupplyCurrentLimitEnable =  true;
		rightConfig.CurrentLimits.StatorCurrentLimit =  80;
		rightConfig.CurrentLimits.SupplyCurrentLimit =  30;
		rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		rightConfig.Voltage.PeakForwardVoltage = 12;
		rightConfig.Voltage.PeakReverseVoltage = -12;

    //Set Configs
		leftMotor.getConfigurator().apply(leftConfig);
		rightMotor.getConfigurator().apply(rightConfig);

    //Set Follower
    leftMotor.setControl(new Follower(PivotConstants.rightPivotID,true));

    //rest position
    rightMotor.setPosition(0);
    leftMotor.setPosition(0);
  }

  public double getPivotAngle(){
    //times 360 to turn rotations into degrees
    //todo: need to tune pivot offset to get optimized zero
    return (pivotEncoder.getAbsolutePosition()*360)-PivotConstants.offset;
  }

  public double getPivotVelocity(){
		return (rightMotor.getVelocity().getValueAsDouble() * PivotConstants.pivotGearRatio);
	}

  public Command setPivotPosistion(Double setpoint){
    ProfiledPIDController controller = new ProfiledPIDController(
      PivotConstants.pivotKp,
      PivotConstants.pivotKi,
      PivotConstants.pivotKd, 
      //todo:these trapizoirdal profiles are extreamly low and need to be tuned up to a usable value
      new TrapezoidProfile.Constraints(50, 50));
      ArmFeedforward feedforward = new ArmFeedforward(0,PivotConstants.pivotKG,0);

      return new FunctionalCommand(
        () -> {controller.reset(getPivotAngle());},
        () -> {
                double speed = feedforward.calculate(getPivotAngle(), getPivotVelocity())
                +controller.calculate(getPivotAngle(),setpoint);
                rightMotor.set(speed);
              },
        (interrupted) -> {},
        () -> controller.atGoal(),
        this);
  }

  public void periodic(){
		SmartDashboard.putNumber("/Pivot/Pivot Posistion", getPivotAngle());
		SmartDashboard.putNumber("/Pivot/Pivot Velocity RPM", getPivotVelocity());

		SmartDashboard.putNumber("/Pivot/Right/Supply Draw", rightMotor.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putNumber("/Pivot/Right/Stator Draw", rightMotor.getStatorCurrent().getValueAsDouble());
		SmartDashboard.putNumber("/Pivot/Right/Temp C*", rightMotor.getDeviceTemp().getValueAsDouble());

		SmartDashboard.putNumber("/Pivot/Left/Supply Draw", leftMotor.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putNumber("/Pivot/Left/Stator Draw", leftMotor.getStatorCurrent().getValueAsDouble());
		SmartDashboard.putNumber("/Pivot/Left/Temp C*", leftMotor.getDeviceTemp().getValueAsDouble());
	}

}