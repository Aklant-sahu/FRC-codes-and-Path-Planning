// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class DriveTrain extends SubsystemBase {

  CANSparkMax leftmotorfront;
   CANSparkMax leftmotorback;
  CANSparkMax rightmotorfront;
   CANSparkMax rightmotorback;
  // SpeedControllerGroup leftmotors;
  DifferentialDrive drive;
  DifferentialDriveOdometry drive2;
  MotorControllerGroup leftmotors;
  MotorControllerGroup rightmotors;
  RelativeEncoder encleftf;
  RelativeEncoder encrightf;
  RelativeEncoder encleftb;
  RelativeEncoder encrightb;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private static final int kCPR = 8192;
  PIDController controller;
  DifferentialDriveOdometry odom_drive;
  AHRS ahrs;
  Pose2d pose;
  SimpleMotorFeedforward feedforward;
  double kV,kS,kA;
  
 

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

    // leftmotorfront.restoreFactoryDefaults();
    encleftf = leftmotorfront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    // rightmotorfront.restoreFactoryDefaults();
    encrightf = rightmotorfront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
    // leftmotorback.restoreFactoryDefaults();
    encleftb = leftmotorback.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    // rightmotorback.restoreFactoryDefaults();
    encrightb = rightmotorback.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    ahrs=new AHRS(SPI.Port.kMXP);
    controller=new PIDController(1.1,0.07,0);
    odom_drive=new DifferentialDriveOdometry(Rotation2d.fromDegrees(-ahrs.getAngle()));
    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    
    

  


  

  

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // put smart dashboard here
    // pose=odom_drive.update(Rotation2d.fromDegrees(-ahrs.getAngle()),(encleftf.getPosition() * Math.PI * Units.inchesToMeters(6)) / 7.31,
    // (encrightf.getPosition() * Math.PI * Units.inchesToMeters(6)) / 7.31);
    SmartDashboard.putNumber("angle POSE",-ahrs.getAngle());
    // SmartDashboard.putNumber("X", pose.getX());
    // SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("Left", encleftf.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31);//
    SmartDashboard.putNumber("Right", encrightf.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31);
    
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
  public void drive1m(double left,double right){
  
    leftmotors.set(controller.calculate((left+right)/2, 1)*0.20);
    rightmotors.set(controller.calculate((left+right)/2, 1)*0.20);
    
  }
  public double leftencpos(){
      return  encleftb.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31;
    // double right =  FR_encoder.getPosition() * Math.PI * Units.inchesToMeters(6)/3.488 * 0.488;
  }
  public double rightencpos(){
    return encrightb.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31;
  }
  public void resetenc(){
    encleftb.setPosition(0);
    encleftf.setPosition(0);
    encrightf.setPosition(0);
    encrightb.setPosition(0);
  }
  public double lim(double setp,double speed,double kp){
    return (speed*0.3)/(setp*kp);
  }
  public void rotatetoangle(){

  }
}