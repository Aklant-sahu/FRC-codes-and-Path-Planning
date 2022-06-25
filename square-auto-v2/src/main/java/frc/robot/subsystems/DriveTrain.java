// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftfront;
  private CANSparkMax rightfront;
  private CANSparkMax leftback;
  private CANSparkMax rightback;
  private MotorController leftmotors;
  private MotorController rightmotors;
  

  private DifferentialDrive drive;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odom_drive;

  private PIDController distPid;
  private PIDController anglePid;
  // private double kp,ki,kd;

  private RelativeEncoder encleft;
  private RelativeEncoder encright;
  private Pose2d pose;
  private Rotation2d rot;

  public AHRS ahrs;
  private double[] targetX;
  private double[] targetY;
  double[] tempx,tempy;
  public double currX, currY, d_theta = 0.0, d = 0.0;
  public double angletotake;
  public double angle, trans_Lmot, trans_Rmot;
  
  


  



  public  DriveTrain() {
    leftfront=new CANSparkMax(Constants.leftfrontid,MotorType.kBrushless);
    leftback=new CANSparkMax(Constants.leftbackid,MotorType.kBrushless);
    rightfront=new CANSparkMax(Constants.rightfrontid,MotorType.kBrushless);
    rightback=new CANSparkMax(Constants.rightbackid,MotorType.kBrushless);

    leftmotors=new MotorControllerGroup(leftfront, leftback);
    rightmotors=new MotorControllerGroup(rightfront, rightback);
    drive=new DifferentialDrive(leftmotors,rightmotors);
    drive.setDeadband(0.05);

    leftback.follow(leftfront);
    rightback.follow(rightfront);
    leftfront.setInverted(false);
    rightfront.setInverted(true);

    encleft=leftfront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    encright=rightfront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    // leftback.follow(leftfront);
    // rightback.follow(rightfront);
    pose=new Pose2d();
    ahrs=new AHRS(SPI.Port.kMXP);
    distPid=new PIDController(Constants.drivekp, Constants.driveki,Constants.drivekd);
    distPid.setTolerance(0.1);
    anglePid=new PIDController(Constants.anglekp, Constants.angleki, Constants.anglekd);
    anglePid.setTolerance(0.5);
    odom_drive=new DifferentialDriveOdometry(getangle());
    targetX=new double[] {Constants.initialX};
    targetY=new double[] {Constants.initialY};
    angletotake=0;
    currX=targetX[0];
    currY=targetY[0];
    // this trajectory is for anticlockwise rotation hence in this angle output would be positive since we taking -ahrs

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose=odom_drive.update(getangle(),leftencpos(),rightencpos());
    SmartDashboard.putNumber("angle",angle);
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("Left", encleft.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31);
    SmartDashboard.putNumber("Right", encright.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31);
    
  }
  public Rotation2d getangle(){
    return Rotation2d.fromDegrees(-ahrs.getAngle());
    // ahrs.getangle gives continuous angle output to 
    //prevent disccontuity
  }
  public double getPosAngle(){
    return pose.getRotation().getDegrees();
  }
  public double getHeading(){
    return ahrs.getYaw();
    //gives angle in -180 to +180 degree range
  }
  public double leftencpos(){
    return  encleft.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31;
  
  }
  public double rightencpos(){
    return encright.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31;
  }
  public void driveWithJoysticks(Joystick joy,double speed){
    drive.arcadeDrive(-joy.getRawAxis(1)*speed,joy.getRawAxis(4)*speed);
  }

 public void arcade(double speed,double power){
  drive.arcadeDrive(speed,power);
 }
  public void driveWith(double speedleft,double speedright){
   leftmotors.set(speedleft);
   rightmotors.set(speedright);
  }
  public void stopMotors(){
    drive.stopMotor();
  }
  public void driveForward(double speed){
    drive.tankDrive(speed,speed);
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    double leftspeed=(leftfront.getEncoder().getVelocity()/7.31)*2*Math.PI*Units.inchesToMeters(3)/60;
    double rightspeed=(rightfront.getEncoder().getVelocity()/7.31)*2*Math.PI*Units.inchesToMeters(3)/60;
    return new DifferentialDriveWheelSpeeds(leftspeed,rightspeed);
  }
  public double orient(double angle){
    pose=odom_drive.update(getangle(),leftencpos(),rightencpos());
   leftmotors.set( -MathUtil.clamp(anglePid.calculate(pose.getRotation().getDegrees(), angle),-Constants.maxanglepidout,Constants.maxanglepidout));
   rightmotors.set(MathUtil.clamp(anglePid.calculate(pose.getRotation().getDegrees(), angle),-Constants.maxanglepidout,Constants.maxanglepidout));
   return (angle-pose.getRotation().getDegrees());
  }
  public double drive1m(double left,double right,double setp){
  
    leftmotors.set(MathUtil.clamp(distPid.calculate(left, setp),-Constants.maxdistpidout,Constants.maxdistpidout));
    rightmotors.set(MathUtil.clamp(distPid.calculate(right, setp),-Constants.maxdistpidout,Constants.maxdistpidout));
    return (setp-left);
    
  }
  public void res(){
    ahrs.reset();
  }
  public void resetenc(){
    encleft.setPosition(0);
    encright.setPosition(0);
    
  }
  public boolean checkDist(){
    return distPid.atSetpoint();
  }
  public boolean checkAng(){
    return anglePid.atSetpoint();
  }


  public double[] pidPath(double x, double y) {
    currX = pose.getX();
    currY = pose.getY();

    angle = -ahrs.getAngle() % 360;
    angletotake = Math.toDegrees(Math.atan2((y - currY), (x - currX)));  //atan2 gives -180 se 180 degrees range with ccw being positive.
    angletotake = Math.toDegrees(Math.atan2(1, 1));

    // Clip the Angle from [-180 to 180] -> [0 to 360]
    angletotake = (angletotake + 720) % 360;
    double d = Math.sqrt(Math.pow((currX - x), 2) + Math.pow((currY - y), 2));
    double d_theta = angletotake - angle;
    double theta_dir = d_theta / Math.abs(d_theta);

    if (Math.abs(d_theta) >= 180) {
      d_theta = 360 - Math.abs(d_theta);
      theta_dir *= -1;
    } else {
      d_theta = Math.abs(d_theta);
      theta_dir *= 1;
    }
    // double angle1=Math.atan2(1,1);
    // System.out.println(angle1);
    // double angle2=Math.atan2(-1,-1); 
    // double angle1=Math.atan2(currY,currX);
    //  
    
    SmartDashboard.putNumber("d", d);
    SmartDashboard.putNumber("x", currX);
    SmartDashboard.putNumber("y", currY);
    SmartDashboard.putNumber("angle to take", angletotake);
    SmartDashboard.putNumber("angle to take with direction", d_theta *
    theta_dir);
    SmartDashboard.putNumber("D Theta", d_theta);
    SmartDashboard.putNumber("Dir", theta_dir);
   
    // SmartDashboard.putNumber("speed",anglePid.calculate(0, d_theta *theta_dir));
    trans_Lmot = distPid.calculate(0, d) - anglePid.calculate(0, d_theta * theta_dir);
    trans_Rmot = distPid.calculate(0, d) + anglePid.calculate(0, d_theta * theta_dir);
    

   
    
    // if (Math.abs(x - currX) > 0.2 || Math.abs(y - currY) > 0.2) {
    //   trans_Lmot = distPid.calculate(0, d) - anglePid.calculate(0, d_theta * theta_dir);
    //   trans_Rmot = distPid.calculate(0, d) + anglePid.calculate(0, d_theta * theta_dir);
    // } else {
    //   trans_Lmot = 0;
    //   trans_Rmot = 0;
    // }

    // Equal Weightage for Both PIDs
    if (Math.abs(trans_Lmot) > Constants.maxdistpidout
        || Math.abs(trans_Rmot) > Constants.maxdistpidout) {
      if (Math.abs(trans_Lmot) > Math.abs(trans_Rmot)) {
        trans_Rmot = Constants.maxdistpidout * trans_Rmot / Math.abs(trans_Lmot);
        trans_Lmot = Constants.maxdistpidout * trans_Lmot / Math.abs(trans_Lmot);
        // MathUtil.F(trans_Lmot, -0.2, 0.2);
      } else {
        trans_Lmot = Constants.maxdistpidout * trans_Lmot / Math.abs(trans_Rmot);
        trans_Rmot = Constants.maxdistpidout * trans_Rmot / Math.abs(trans_Rmot);
        // MathUtil.clamp(trans_Rmot, -0.2, 0.2);
      }

    }
    double speed[] = { trans_Lmot, trans_Rmot };
    // double speed[] = { Constants.maxdistpidout,
    // Constants.maxdistpidout };

    return speed;
    // Ang_Lmot=
    // Ang_Rmot=
  }
  public double[] pidForward(double x, double y) {
    currX = pose.getX();
    currY = pose.getY();

    double d = Math.sqrt(Math.pow((currX - x), 2) + Math.pow((currY - y), 2));
 
    
    SmartDashboard.putNumber("d", d);
    SmartDashboard.putNumber("x", currX);
    SmartDashboard.putNumber("y", currY);
  
 
    // SmartDashboard.putNumber("speed",anglePid.calculate(0, d_theta *theta_dir));
    trans_Lmot = distPid.calculate(0, d);
    trans_Rmot = distPid.calculate(0, d);

    // Equal Weightage for Both PIDs
    if (Math.abs(trans_Lmot) > Constants.maxdistpidout
        || Math.abs(trans_Rmot) > Constants.maxdistpidout) {
      if (Math.abs(trans_Lmot) > Math.abs(trans_Rmot)) {
        trans_Rmot = Constants.maxdistpidout * trans_Rmot / Math.abs(trans_Lmot);
        trans_Lmot = Constants.maxdistpidout * trans_Lmot / Math.abs(trans_Lmot);
        // MathUtil.F(trans_Lmot, -0.2, 0.2);
      } else {
        trans_Lmot = Constants.maxdistpidout * trans_Lmot / Math.abs(trans_Rmot);
        trans_Rmot = Constants.maxdistpidout * trans_Rmot / Math.abs(trans_Rmot);
        // MathUtil.clamp(trans_Rmot, -0.2, 0.2);
      }

    }
    double speed[] = { trans_Lmot, trans_Rmot };
    // double speed[] = { Constants.maxdistpidout,
    // Constants.maxdistpidout };

    return speed;
    // Ang_Lmot=
    // Ang_Rmot=
  }
  public double[] pidRotate(double x, double y) {
    currX = pose.getX();
    currY = pose.getY();

    angle = -ahrs.getAngle() % 360;
    angletotake = Math.toDegrees(Math.atan2((y - currY), (x - currX)));  //atan2 gives -180 se 180 degrees range with ccw being positive.
    angletotake = Math.toDegrees(Math.atan2(1, 1));

    // Clip the Angle from [-180 to 180] -> [0 to 360]
    angletotake = (angletotake + 720) % 360;
    
    double d_theta = angletotake - angle;
    double theta_dir = d_theta / Math.abs(d_theta);

    if (Math.abs(d_theta) >= 180) {
      d_theta = 360 - Math.abs(d_theta);
      theta_dir *= -1;
    } else {
      d_theta = Math.abs(d_theta);
      theta_dir *= 1;
    }
   
   
    // SmartDashboard.putNumber("speed",anglePid.calculate(0, d_theta *theta_dir));
    trans_Lmot = - anglePid.calculate(0, d_theta * theta_dir);
    trans_Rmot =  + anglePid.calculate(0, d_theta * theta_dir);
    


    // Equal Weightage for Both PIDs
    if (Math.abs(trans_Lmot) > Constants.maxdistpidout
        || Math.abs(trans_Rmot) > Constants.maxdistpidout) {
      if (Math.abs(trans_Lmot) > Math.abs(trans_Rmot)) {
        trans_Rmot = Constants.maxdistpidout * trans_Rmot / Math.abs(trans_Lmot);
        trans_Lmot = Constants.maxdistpidout * trans_Lmot / Math.abs(trans_Lmot);
        // MathUtil.F(trans_Lmot, -0.2, 0.2);
      } else {
        trans_Lmot = Constants.maxdistpidout * trans_Lmot / Math.abs(trans_Rmot);
        trans_Rmot = Constants.maxdistpidout * trans_Rmot / Math.abs(trans_Rmot);
        // MathUtil.clamp(trans_Rmot, -0.2, 0.2);
      }

    }
    double speed[] = { trans_Lmot, trans_Rmot };
    // double speed[] = { Constants.maxdistpidout,
    // Constants.maxdistpidout };

    return speed;
    // Ang_Lmot=
      // Ang_Rmot=
  }
  public double getCurrX() {
    return currX;
  }

  public double getCurrY() {
    return currY;
  }

  public double get_angle() {
    return angle;
  }
  public double getNextX(int counterx){
      return tempx[counterx];
  }
  public double getNextY(int countery){
      return tempy[countery];
  }
  public void updatePose(){
 pose=odom_drive.update(getangle(),leftencpos(),rightencpos());
  }
  public void setPose(){
    pose=odom_drive.update(getangle() ,Constants.initialX,Constants.initialY);
  }



}