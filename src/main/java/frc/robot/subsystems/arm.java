// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class arm extends SubsystemBase {
    /*change based on port*/ 
  private WPI_TalonSRX arm= new WPI_TalonSRX(4);

  /** Creates a new arm. */
  public arm() {
    //test if needs to be inverted
    arm.setInverted(false);
    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    arm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    arm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

  }
  public double getAAngle(){
    //need to ask about gear ratio
    return ((getATicks())/4096)*360;
  }
  public double getATicks(){
    return arm.getSelectedSensorPosition();
  }
  public void setASpeed(double speed){
    arm.set(ControlMode.PercentOutput, speed);
  }
  public void StopA(boolean brakes){
    if(brakes){
      arm.setNeutralMode(NeutralMode.Brake);
    }
    else{
      arm.setNeutralMode(NeutralMode.Coast);
    }
  }
  public void resetA(){
    arm.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
