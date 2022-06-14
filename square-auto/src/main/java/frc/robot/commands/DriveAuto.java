// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveAuto extends CommandBase {
  /** Creates a new DriveAuto. */
  private final DriveTrain driveTrain;
  public boolean todo;
  public double setp=0;
  public float angle;
  boolean ret;
  Timer timer;
  /** Creates a new DriveWithJoysticks. */
  public DriveAuto(DriveTrain dt,boolean tod,double set,float angl,boolean ret) {
    driveTrain=dt;
    addRequirements(driveTrain);
    todo=tod;
    this.setp=set;
    angle=angl;
    // ret=ret;
    timer=new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ret=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(todo==true){
     
      // timer.reset();
      // timer.start();
      // while(timer.get()<6){
      double err=driveTrain.orient(angle);
      if(err<0.02*angle ){
        ret=true;
      }
      System.out.println("rotating robot");
      
    // }
    // ret=true;
    driveTrain.resetenc();
    
     
    }
    else if(todo==false){
      
      // timer.reset();
      // timer.start();
      // while(timer.get()<3){
      double left=driveTrain.leftencpos();
      double right=driveTrain.rightencpos();
      double err=driveTrain.drive1m(left, right,this.setp);
      if(err<0.06*setp ){
        ret=true;
      }
      System.out.println("moving robot straight");
      // }
      // ret=true;
    }
    // driveTrain.res();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ret;
  }
}
