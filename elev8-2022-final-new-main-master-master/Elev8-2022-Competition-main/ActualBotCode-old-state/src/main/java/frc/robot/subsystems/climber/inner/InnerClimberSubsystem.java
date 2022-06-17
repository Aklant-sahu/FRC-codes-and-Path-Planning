// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.inner;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.ClimberConstants.InnerClimberConstants;

public class InnerClimberSubsystem extends SubsystemBase {
  private final CANSparkMax in_LC, in_RC;
  private final RelativeEncoder in_LC_encoder, in_RC_encoder;
  private final MotorControllerGroup in_C;

  /** Creates a new InnerClimberSubsystem. */
  public InnerClimberSubsystem() {
    this.in_LC = new CANSparkMax(31, MotorType.kBrushless);
    this.in_RC = new CANSparkMax(32, MotorType.kBrushless);
    this.in_LC.setInverted(false);
    this.in_RC.setInverted(true);
    this.in_LC_encoder = this.in_LC.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DrivingConstants.neoCountsPerRevolution);
    this.in_RC_encoder = this.in_RC.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        DrivingConstants.neoCountsPerRevolution);
    this.in_LC.setIdleMode(IdleMode.kBrake);
    this.in_RC.setIdleMode(IdleMode.kBrake);

    this.in_C = new MotorControllerGroup(this.in_LC, this.in_RC);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void inners_Break() {
    this.in_LC.setIdleMode(IdleMode.kBrake);
    this.in_RC.setIdleMode(IdleMode.kBrake);
  }

  public void inners_Coast() {
    this.in_LC.setIdleMode(IdleMode.kCoast);
    this.in_RC.setIdleMode(IdleMode.kCoast);
  }

  public void setInnerSpeed(double outers) {
    this.in_C.set(outers * InnerClimberConstants.speedMultiplier);
  }

  public void setLeft_IC_BasePosition() {
    this.in_LC_encoder.setPosition(0.0);
    this.in_RC_encoder.setPosition(0.0);
  }

  public double getLeft_IC_Position() {
    return this.in_LC_encoder.getPosition();
  }

  public double getLeft_RC_Position() {
    return this.in_RC_encoder.getPosition();
  }

  public void setSpeed(double[] speeds) {
    this.in_LC.set(speeds[0]);
    this.in_RC.set(speeds[1]);
  }
}
