// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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
  Encoder leftEncoder;
  Encoder rightEncoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  // private static final int kCPR = 8192;
  PIDController controller;
  DifferentialDriveOdometry odom_drive;
  AHRS ahrs;
  Pose2d pose;
  SimpleMotorFeedforward feedforward;
  double kV,kS,kA;
  DifferentialDriveKinematics kinematics;
  PIDController anglePidController;
  CANSparkMax intake;
  private final  double kEncoderTicktoMeter = 1.0 / 4096.0 * 0.128 * Math.PI;
  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;
  private ADXRS450_GyroSim m_gyroSim;
 

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftmotorfront=new CANSparkMax(11,  MotorType.kBrushless);
    intake=new CANSparkMax(5,  MotorType.kBrushless);
    // leftmotorfront.setInverted(false);
    leftmotorback=new CANSparkMax(12,  MotorType.kBrushless);
    // leftmotorback.setInverted(false);
    rightmotorfront=new CANSparkMax(22,  MotorType.kBrushless);
    // rightmotorfront.setInverted(true);
    rightmotorback=new CANSparkMax(21,  MotorType.kBrushless);
    // rightmotorback.setInverted(true);

    leftEncoder = new Encoder(0, 1);
    rightEncoder = new Encoder(2, 3);

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
    controller=new PIDController(1.2,0,0);
    anglePidController=new PIDController(0.8, 0, 0);
    // anglePidController.setIntegratorRange(-0.5, 0.5);
    // anglePidController.setTolerance(2f);
    
    
    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    kinematics=new DifferentialDriveKinematics(1);
    odom_drive=new DifferentialDriveOdometry(getHeading());

    leftmotorback.follow(leftmotorfront);
    rightmotorback.follow(rightmotorfront);
    leftmotorfront.setInverted(false);
    rightmotorfront.setInverted(true);

    // Sets the distance per pulse for the encoders
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderTicktoMeter);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderTicktoMeter);
    resetEncoders();
    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);
    }
  }
  double getEncoderMeters() {
    return (leftEncoder.get() * -rightEncoder.get())/2 * kEncoderTicktoMeter;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // put smart dashboard here
    pose=odom_drive.update(getHeading(),leftencpos(),rightencpos());
    SmartDashboard.putNumber("angle POSE",-ahrs.getAngle());
    // SmartDashboard.putNumber("X", pose.getX());
    // SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("Left", encleftf.getPosition() * Math.PI * Units.inchesToMeters(6)/7.);//
    SmartDashboard.putNumber("Right", encrightf.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31);
    m_gyro.getAngle();
    m_gyroSim = new ADXRS450_GyroSim(m_gyro);
    m_fieldSim.setRobotPose(getPose());
  }
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }
  public DifferentialDriveWheelSpeeds getSpeeds(){
    double leftspeed=(leftmotorfront.getEncoder().getVelocity()/7.31)*2*Math.PI*Units.inchesToMeters(3)/60;
    double rightspeed=(rightmotorfront.getEncoder().getVelocity()/7.31)*2*Math.PI*Units.inchesToMeters(3)/60;
    return new DifferentialDriveWheelSpeeds(leftspeed,rightspeed);
  }
  public double leftencpos(){
    return  encleftb.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31;
  
}
public double rightencpos(){
  return encrightb.getPosition() * Math.PI * Units.inchesToMeters(6)/7.31;
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
  public void resetenc(){
    encleftb.setPosition(0);
    encleftf.setPosition(0);
    encrightf.setPosition(0);
    encrightb.setPosition(0);
  }
  public double orient(double angle){
   pose=odom_drive.update(getHeading(),leftencpos(),rightencpos());
   leftmotors.set( -anglePidController.calculate(pose.getRotation().getDegrees(), angle)/250);
   rightmotors.set(anglePidController.calculate(pose.getRotation().getDegrees(), angle)/250);
   return (angle-pose.getRotation().getDegrees());
  }
  public double drive1m(double left,double right,double setp){
  
    leftmotors.set(controller.calculate(left, setp)*0.35);
    rightmotors.set(controller.calculate(right, setp)*0.35);
    return (setp-left);
    
  }
  public void res(){
    ahrs.reset();
  }
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
  // Returns the currently-estimated pose of the robot.
  public Pose2d getPose() {
    return odom_drive.getPoseMeters();
  }


  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(
        leftmotors.get() * RobotController.getBatteryVoltage(),
        rightmotors.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

}