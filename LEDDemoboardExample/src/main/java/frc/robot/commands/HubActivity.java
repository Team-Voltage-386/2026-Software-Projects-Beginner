// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LightSubsystem;

/** 
 * Will display whether the hub is active at a given time.
 */
public class HubActivity extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Timer timer = new Timer();
  private LightSubsystem LightSubsystem;
  private int counter = 0; // counter starts at 0
  private boolean hubIsActive = true;
  private boolean isAutoAhead = true; // This is a placeholder. Replace with logic to determine whether we are actually ahead! (Even though we totally will be every time)

  // prerecorded times for hub activity. Each time the timer reaches the value in the array, the hub toggles activity states.
  private int[] timesWinning = {20, 50, 80, 110}; // Test values only so far. Will replace with actual values.
  private int[] timesLosing = {50, 80, 110, 140};

  /**
   * Constructor for the CycleLED Command class.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HubActivity(LightSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LightSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isAutoAhead) {
        if (timer.get() > timesWinning[counter]) {
          counter++;
          hubIsActive = !hubIsActive;
          System.out.println("Hub is " + hubIsActive);
        }
      }
    else {
      if (timer.get() > timesLosing[counter]) {
        counter++;
        hubIsActive = !hubIsActive;
        System.out.println("Hub is " + hubIsActive);
      }
    }
    if (hubIsActive) {
      LightSubsystem.changeAllLEDColor(0, 255, 0);
    }
    else {
      LightSubsystem.changeAllLEDColor(255, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= 4; // Returns true when the game ends.
  }
  public boolean hubIsActive() {
    return hubIsActive;
  }

}
