// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
  private int leftDeviceID=33;
  private CANSparkMax leftmotor=new CANSparkMax(leftDeviceID, MotorType.kBrushless);
 
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Motor speed",joy1.getY());
  
  }
  public  void driveWithController(Joystick joy,double leftspeed){
    leftmotor.set(joy.getRawAxis(1)*leftspeed);
    // leftmotor.set(100);
    SmartDashboard.putNumber("Motor speed",joy.getRawAxis(1));
        
  }

  public  void driveauto(){
    leftmotor.set(0.3);
    // leftmotor.set(100);
   
        
  }
  public  void stopMotors(){
    leftmotor.set(0);
    // leftmotor.set(100);
   
        
  }
}
