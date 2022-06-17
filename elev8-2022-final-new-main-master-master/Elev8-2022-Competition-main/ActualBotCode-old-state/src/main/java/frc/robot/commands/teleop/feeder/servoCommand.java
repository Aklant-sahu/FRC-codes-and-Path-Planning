// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.servo.ServoFeederSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class servoCommand extends CommandBase {
  /** Creates a new servoCommand. */
  private boolean servo_temp = true;
  ServoFeederSubsystem servoFeederSubsystem;

  public servoCommand(ServoFeederSubsystem servoFeederSubsystem) {
    this.servoFeederSubsystem = servoFeederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(servoFeederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.servoFeederSubsystem.setServoSpeed(FeederConstants.positionAngle);
    SmartDashboard.putBoolean("Servo up?", servo_temp);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("servoreading", servoFeederSubsystem.getPosition());
    // if (servoFeederSubsystem.getPosition() == 0) {
    // SmartDashboard.putBoolean("Servo up?", false);
    // } else {
    
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.servoFeederSubsystem.setServoSpeed(FeederConstants.initialAngle);
    servo_temp = false;
    SmartDashboard.putBoolean("s?", servo_temp);
    // SmartDashboard.putBoolean("Servo up?", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !RobotContainer.joyD.getRawButton(4);
    return false;
  }
}
