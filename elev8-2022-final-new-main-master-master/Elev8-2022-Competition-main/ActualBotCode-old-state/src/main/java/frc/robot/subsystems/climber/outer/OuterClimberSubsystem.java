// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.outer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants.OuterClimberConstants;

public class OuterClimberSubsystem extends SubsystemBase {
  private final WPI_TalonFX out_LC, out_RC;
  private final MotorControllerGroup out_C;
  // private final PIDController feedback_roll;

  /** Creates a new OuterClimberSubsystem. */
  public OuterClimberSubsystem() {
    this.out_LC = new WPI_TalonFX(5);
    this.out_RC = new WPI_TalonFX(6);
    this.out_LC.setInverted(false);
    this.out_RC.setInverted(true);
    this.out_LC.setNeutralMode(NeutralMode.Brake);
    this.out_RC.setNeutralMode(NeutralMode.Brake);

    this.out_C = new MotorControllerGroup(this.out_LC, this.out_RC);
    // feedback_roll = new PIDController(0.006, 0, 0.0001);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOuterSpeed(double inners) {
    this.out_C.set(inners * OuterClimberConstants.speedMultiplier);
    // this.out_C.set(inners*feedback_roll.calculate(DriveSubsystem.navx.getRoll(),
    // 0));
  }

  public double getLeft_OC_Position() {
    return this.out_LC.getSelectedSensorPosition();
  }

  public void setLeft_OC_BasePosition() {
    this.out_LC.setSelectedSensorPosition(0.0);
  }

  public double getRight_OC_Position() {
    return this.out_RC.getSelectedSensorPosition();
  }

  public void setRight_OC_BasePosition() {
    this.out_RC.setSelectedSensorPosition(0.0);
  }

  public void setSpeed(double[] speeds) {
    this.out_LC.set(speeds[0]);
    this.out_RC.set(speeds[1]);
  }
}
