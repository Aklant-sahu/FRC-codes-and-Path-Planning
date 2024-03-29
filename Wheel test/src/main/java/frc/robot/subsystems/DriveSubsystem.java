// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveSubsystem extends SubsystemBase {

  private Spark leftmotor1=new Spark(0);

  private CANSparkMax left=new CANSparkMax(1, MotorType.kBrushless);
  // private Spark leftmotor2=new Spark(1);
  // private Spark rightmotor1=new Spark(2);
  // private Spark rightmotor2=new Spark(3);
  /** Creates a new ExampleSubsystem. */
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Base motor","The motor is running");
  }

  public void setMotors(int leftspeed,int rightspeed)
{
  leftmotor1.set(leftspeed);
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
