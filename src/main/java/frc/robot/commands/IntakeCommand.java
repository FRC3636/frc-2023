// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw.ClawPosition;
import frc.robot.subsystems.Claw;


public class IntakeCommand extends CommandBase {

  private Claw claw;
  private ClawPosition position;


  /** Creates a new Claw. */
  public IntakeCommand(Claw claw, ClawPosition object) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    position = object;
    addRequirements(this.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setClawPosition(position);
    claw.runRollers(Constants.Claw.ROLLER_IN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.runRollers(Constants.Claw.ROLLER_OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
