// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class ServoFeederSubsystem extends SubsystemBase {
  private final Servo stopper;

  /** Creates a new ServoFeederSubsystem. */
  public ServoFeederSubsystem() {
    this.stopper = new Servo(FeederConstants.servo_port);
    this.stopper.setAngle(FeederConstants.positionAngle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("servo speed", this.stopper.get());
    if (this.stopper.get() == 0) {
      SmartDashboard.putBoolean("Servo up?", false);
    } else {
      SmartDashboard.putBoolean("Servo up?", true);
    }
    // This method will be called once per scheduler run
  }

  public double getPosition() {
    return this.stopper.getAngle();
  }

  public void setServoSpeed(double angle) {
    // double speedmotor = this.stopper.get();
    // if (Math.abs(speedmotor) < FeederConstants.deadband) {
    // double now = Timer.getFPGATimestamp();
    // while (now < 0.6) {
    // if (now == 0.2)
    // this.feeder.set(FeederConstants.feederSpeed / 3);
    // if (now == 0.4)
    // this.feeder.set(FeederConstants.feederSpeed / 2);
    // }
    // this.feeder.set(FeederConstants.feederSpeed);
    this.stopper.setAngle(angle);
    // System.out.println("Timer : " + now);
    // } else {
    // // this.feeder.set(0);
    // this.stopper.setAngle(FeederConstants.initialAngle);

    // }
  }
}
