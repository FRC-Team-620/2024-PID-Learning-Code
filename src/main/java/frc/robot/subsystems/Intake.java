// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.DEVICE_ID, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public CANSparkMax getMotor() {
    return intakeMotor;
  }

  public RelativeEncoder getMotorEncoder() {
    return intakeMotor.getEncoder();
  }

  public double getEncoderPosition() {
    return intakeMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("MotorEncoderVal", getEncoderPosition());
  }

  public void zeroOdom() {
    getMotorEncoder().setPosition(0);
  }

  public void setSpinSpeed(double speed) {
    intakeMotor.set(speed);
  }
}
