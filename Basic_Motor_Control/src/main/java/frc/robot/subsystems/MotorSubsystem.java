package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.MotorRunCommand;

public class MotorSubsystem extends SubsystemBase {
  private final NetworkTable table;
  // Set up the widgets on the dashboard
  private DoubleEntry m_requestedVoltage;
  private DoublePublisher m_voltageOutputGenericEntry;
  private DoublePublisher m_currentOutputGenericEntry;
  private DoubleEntry m_incDecAmountGenericEntry;
  private BooleanEntry m_incrementVoltageGenericEntry;
  private BooleanEntry m_decrementVoltageGenericEntry;
  private BooleanEntry m_doRunMotorsGenericEntry;
  private BooleanPublisher m_doRunMotorsOutputGenericEntry;

  private SparkMax m_motor;
  private boolean m_askedToRunMotors;
  private Command m_motorRunCommand;
  private static final double EPSILON = 0.001;

  public MotorSubsystem(NetworkTableInstance nt) {

    // Set up motor state
    this.m_motor = new SparkMax(Constants.Motor.kCANID, MotorType.kBrushless);
    this.m_askedToRunMotors = false;
    this.m_motorRunCommand = new MotorRunCommand(this);

    table = nt.getTable(getName());
    
    m_voltageOutputGenericEntry = table.getDoubleTopic("Output Voltage").publish();
    m_currentOutputGenericEntry = table.getDoubleTopic("Motor Current").publish();
    m_requestedVoltage = table.getDoubleTopic("Input Voltage").getEntry(0.0, PubSubOption.sendAll(true));
    m_requestedVoltage.set(0.0);
    m_incDecAmountGenericEntry = table.getDoubleTopic("Inc_Dec Amount").getEntry(0.0, PubSubOption.sendAll(true));
    m_incDecAmountGenericEntry.set(0.0);

    m_incrementVoltageGenericEntry = table.getBooleanTopic("INC").getEntry(false);
    m_incrementVoltageGenericEntry.set(false);// needed to make the entry show up in elastic
    m_decrementVoltageGenericEntry = table.getBooleanTopic("DEC").getEntry(false);
    m_decrementVoltageGenericEntry.set(false);// needed to make the entry show up in elastic
    m_doRunMotorsGenericEntry = table.getBooleanTopic("Run Motors?").getEntry(false);
    m_doRunMotorsGenericEntry.set(false);// needed to make the entry show up in elastic
    m_doRunMotorsOutputGenericEntry = table.getBooleanTopic("RunMotors Print").publish();

    // when the robot is not disabled and you've clicked the button on Elastic
    // to run the motors, the motor voltage is set based on the value on
    // Elastic otherwise, the voltage is set to zero

    // Note that the voltage is set to zero when the robot is disabled
    new Trigger(() -> this.m_askedToRunMotors).and(this::doRunMotor)
        .whileTrue(m_motorRunCommand).onFalse(
            runOnce(
                () -> {
                  this.setMotorVoltage(0.0);
                }).ignoringDisable(true));

    // sets a visual indicator on Elastic when the motor voltage is greater
    // than EPSILON
    new Trigger(
        () -> {
          return Math.abs(this.getMotorVoltage()) < EPSILON;
        }).onTrue(
            Commands.runOnce(
                () -> {
                  this.m_doRunMotorsOutputGenericEntry.set(false);
                }).ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                () -> {
                  this.m_doRunMotorsOutputGenericEntry.set(true);
                }).ignoringDisable(true));

    // When the increment voltage button is pressed, the voltage is incremented
    //
    // Note: In Elastic buttons are toggles, i.e press once and it turns on.
    // Press again to turn it off. This trigger makes it so that press once it turns
    // on (and increments) and then it turns it off automatically so the next time
    // you press the button it'll increment again.

    new Trigger(() -> {
      return this.m_incrementVoltageGenericEntry.get(false);
    }).onTrue(Commands.runOnce(() -> this.incrementVoltageGenericEntry()).andThen(Commands.runOnce(() -> {
      this.m_incrementVoltageGenericEntry.set(false);
    })).ignoringDisable(true));

    // Decrement the voltage (see comment above)
    new Trigger(() -> {
      return this.m_decrementVoltageGenericEntry.get(false);
    }).onTrue(Commands.runOnce(() -> this.decrementVoltageGenericEntry()).andThen(Commands.runOnce(() -> {
      this.m_decrementVoltageGenericEntry.set(false);
    })).ignoringDisable(true));
  }

  /**
   * reads the voltage entered by the user
   * 
   * @return the voltage. If the entered voltage is less than EPSILON it returns
   *         zero
   */
  public double getVoltage() {
    double res = this.m_requestedVoltage.get(0);
    if (Math.abs(res) < EPSILON) {
      return 0.0;
    } else {
      return res;
    }
  }

  /**
   * Checks Elastic button to start the motor
   * 
   * @return the state of the button
   */
  public boolean doRunMotor() {
    boolean runMotorInput = this.m_doRunMotorsGenericEntry.get(false);
    m_doRunMotorsOutputGenericEntry.set(runMotorInput);
    return runMotorInput;
  }

  /**
   * this will read the actual voltage that the motor reports which may be
   * slightly different from the voltage requested.
   * 
   * @return
   */
  public double getMotorVoltage() {
    return this.m_motor.getAppliedOutput();

  }
  public double getMotorCurrent() {
    return this.m_motor.getOutputCurrent();

  }

  /**
   * Sets the voltage of the motor
   * 
   * @param voltage
   */
  public void setMotorVoltage(double voltage) {
    this.m_motor.setVoltage(voltage);
  }

  /**
   * reads the value from Elastic and ensures it's between 0-1 (in order to
   * limit how much the user can increment the voltage)
   * 
   * @return the clamped value
   */
  public double getAndUpdateIncDecAmount() {
    double val = this.m_incDecAmountGenericEntry.get(0.0);
    double clampedVal = MathUtil.clamp(val, 0.0, 1.0);
    this.m_incDecAmountGenericEntry.set(clampedVal);
    return clampedVal;
  }

  public void incrementVoltageGenericEntry() {
    this.m_requestedVoltage.set(this.getVoltage() + this.getAndUpdateIncDecAmount());
  }

  public void decrementVoltageGenericEntry() {
    this.m_requestedVoltage.set(this.getVoltage() - this.getAndUpdateIncDecAmount());
  }

  public void startMotor() {
    this.m_askedToRunMotors = true;
  }

  public void stopMotor() {
    this.m_askedToRunMotors = false;
  }

  /**
   * updates the Elastic object with the motor voltage
   */
  @Override
  public void periodic() {
    m_voltageOutputGenericEntry.set(this.getMotorVoltage());
    m_currentOutputGenericEntry.set(this.getMotorCurrent());
  }
}
