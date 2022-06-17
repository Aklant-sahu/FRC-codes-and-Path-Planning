// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.servo.ServoFeederSubsystem;

public class ServoFeederCommand extends CommandBase {

  private ServoFeederSubsystem servoFeederSubsystem;
  // private boolean servo_temp = false;

  /** Creates a new ServoFeederCommand. */
  public ServoFeederCommand(ServoFeederSubsystem servoFeederSubsystem) {
    this.servoFeederSubsystem = servoFeederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.servoFeederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.servoFeederSubsystem.setServoSpeed(FeederConstants.positionAngle);
    // servo_temp = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putBoolean("Ser?", servo_temp);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.servoFeederSubsystem.setServoSpeed(FeederConstants.initialAngle);
    // SmartDashboard.putBoolean("Servo up?", false);
    // servo_temp = false;
    // SmartDashboard.putBoolean("Ser?", servo_temp);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
