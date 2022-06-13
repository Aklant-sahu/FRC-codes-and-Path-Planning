// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class navx extends SubsystemBase {
  AHRS ahrs;
  DifferentialDriveOdometry m_odometry ;

  /** Creates a new navx. */
  public navx() {
    ahrs=new AHRS(SPI.Port.kMXP);
  //   m_odometry= new DifferentialDriveOdometry(
  // getGyroHeading(), new Pose2d(0, 0, new Rotation2d()));
    
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("angle(yaw)",getangle());
    // SmartDashboard.putNumber("dispx",getdispx());
    // SmartDashboard.putNumber("dispy",getdispy());
    // SmartDashboard.putNumber("compass heading",getcomphead());
  }
  public void res(){
    ahrs.reset();
  }
  public double getangle(){
    return ahrs.getAngle();
  }
  public double getdispx(){
    return ahrs.getDisplacementX();
  }
  public double getdispy(){
    return ahrs.getDisplacementY();
  }
  public double getcomphead(){
    return ahrs.getCompassHeading();
  }
  
  
}

