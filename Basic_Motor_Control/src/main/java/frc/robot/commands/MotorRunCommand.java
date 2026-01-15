package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class MotorRunCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final MotorSubsystem m_subsystem;

  public MotorRunCommand(MotorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // reads the voltage value requested on Elastic and sets the motor voltage
    this.m_subsystem.setMotorVoltage(this.m_subsystem.getVoltage());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // never returns true because the Trigger in the subsystem will cancel the
    // command
    return false;
  }
}
