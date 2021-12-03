// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.controller.PIDController;

public class MoveAngle extends CommandBase {
  //constants to be determined
  PIDController pidController;
  private double kp=.1;
  private double ki=.1;
  private double kd=.1;
  double angle;

  public MoveAngle(double angle) {
    this.angle=angle;
    pidController= new PIDController(kp, ki, kd);
    addRequirements(RobotContainer.getArm());

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    pidController.reset();
    pidController.setTolerance(2.0);
    RobotContainer.getArm().brakeArm(false);
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double speed = pidController.calculate(RobotContainer.getArm().getAAngle(), angle);
    RobotContainer.getArm().setASpeed(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getArm().brakeArm(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

}
