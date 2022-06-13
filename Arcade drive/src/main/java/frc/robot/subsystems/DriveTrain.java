// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {

  CANSparkMax leftmotorfront;
   CANSparkMax leftmotorback;
  CANSparkMax rightmotorfront;
   CANSparkMax rightmotorback;
  // SpeedControllerGroup leftmotors;
  DifferentialDrive drive;
  MotorControllerGroup leftmotors;
  MotorControllerGroup rightmotors;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftmotorfront=new CANSparkMax(11,  MotorType.kBrushless);
    leftmotorfront.setInverted(false);
    leftmotorback=new CANSparkMax(12,  MotorType.kBrushless);
    leftmotorback.setInverted(false);
    rightmotorfront=new CANSparkMax(22,  MotorType.kBrushless);
    rightmotorfront.setInverted(true);
    rightmotorback=new CANSparkMax(21,  MotorType.kBrushless);
    rightmotorback.setInverted(true);
    leftmotors=new MotorControllerGroup(leftmotorfront,leftmotorback);
    rightmotors=new MotorControllerGroup(rightmotorfront,rightmotorback);
    drive=new DifferentialDrive(leftmotors, rightmotors);
    drive.setDeadband(0.05);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // put smart dashboard here
  }

  public void driveWithJoysticks(Joystick joy,double speed){
    drive.arcadeDrive(-joy.getRawAxis(1)*speed,joy.getRawAxis(4)*speed);
  }
  public void stopMotors(){
    drive.stopMotor();
  }
  public void driveForward(double speed){
    drive.tankDrive(speed,speed);
  }
}
