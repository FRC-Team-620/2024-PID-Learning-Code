// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class SetMotorPositionJoystick extends Command {
  Intake intakeSubsystem;
  CommandXboxController controller;

  /** Creates a new SetMotorPositionJoystick. */
  public SetMotorPositionJoystick(Intake intakeSubsystem, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.controller = controller;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get current joystick and system position
    double desiredPos = this.getJoystickAsAngle();
    double currentPos = this.intakeSubsystem.getEncoderPosition();

    // calculate error
    double error = desiredPos - currentPos;
    double actualError = MathUtil.inputModulus(error, -180, 180);

    // calculate proportional controller output
    double pOutput = MathUtil.clamp(
        Constants.ARM_K_proportional * actualError,
        -Constants.ARM_MAX_OUTPUT,
        Constants.ARM_MAX_OUTPUT);

    // set motor output
    intakeSubsystem.setSpinSpeed(pOutput);
  }

  // returns joystick angle as degrees (-180 - 180)
  private double getJoystickAsAngle() {
    // code here
    double xVal = MathUtil.applyDeadband(this.controller.getLeftX(), 0.05);
    double yVal = MathUtil.applyDeadband(this.controller.getLeftY(), 0.05);

    double stickToRads = Math.atan2(xVal, -yVal);

    double stickToDeg = Units.radiansToDegrees(stickToRads);
    SmartDashboard.putNumber("stickToDegrees", stickToDeg);
    return stickToDeg;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setSpinSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
