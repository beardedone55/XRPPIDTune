// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 585.0;
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private double maxSpeed = 20;
  private double kS = 0.0;
  private double kV = 0.0;

  private double feedbackTimeConstant = 0.0;
  private LinearFilter feedbackFilterLeft;
  private LinearFilter feedbackFilterRight;

  private PIDController pid = new PIDController(0.0,0.0,0.0);

  private double commandedLeftSpeed = 0.0;
  private double commandedRightSpeed = 0.0;

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Max Speed", ()->maxSpeed, (a)->maxSpeed = a);
    builder.addDoubleProperty("kS", ()->kS, (a)->kS = a);
    builder.addDoubleProperty("kV", ()->kV, (a)->kV = a);
    builder.addDoubleProperty("kP", pid::getP, pid::setP);
    builder.addDoubleProperty("kD", pid::getD, pid::setD);
    builder.addDoubleProperty("kI", pid::getI, pid::setI);
    builder.addDoubleProperty("Left Speed", m_leftEncoder::getPeriod, null);
    builder.addDoubleProperty("Right Speed", m_rightEncoder::getPeriod, null);
    builder.addDoubleProperty("Left Encoder Count", m_leftEncoder::get, null);
    builder.addDoubleProperty("Right Encoder Count", m_rightEncoder::get, null);
    builder.addDoubleProperty("Encoder Distance Per Pulse", m_rightEncoder::getDistancePerPulse, null);
    builder.addDoubleProperty("Commanded Left Speed", ()->commandedLeftSpeed, null);
    builder.addDoubleProperty("Commanded Right Speed", ()->commandedRightSpeed, null);
    builder.addDoubleProperty("Feedback Filter Time Constant", ()->feedbackTimeConstant, this::setFeedBackFilterTimeConstant);
  }

  public void setFeedBackFilterTimeConstant(double timeConstant) {
    feedbackTimeConstant = timeConstant;
    if(timeConstant == 0.0) {
      feedbackFilterLeft = null;
      feedbackFilterRight = null;
    } else {
      feedbackFilterLeft = LinearFilter.singlePoleIIR(timeConstant, 0.02);
      feedbackFilterRight = LinearFilter.singlePoleIIR(timeConstant, 0.02);
    }
  }

   /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_leftEncoder.setSamplesToAverage(20);
    m_rightEncoder.setSamplesToAverage(20);
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, true);
    setSpeeds(wheelSpeeds.left * maxSpeed, wheelSpeeds.right * maxSpeed);
  }

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV);

    double leftSpeedCurrent = m_leftEncoder.getRate();
    double rightSpeedCurrent = m_rightEncoder.getRate();
    if(feedbackFilterLeft != null) {
      leftSpeedCurrent = feedbackFilterLeft.calculate(leftSpeedCurrent);
      rightSpeedCurrent = feedbackFilterRight.calculate(rightSpeedCurrent);
    }

    commandedLeftSpeed = ff.calculate(leftSpeed) + pid.calculate(leftSpeedCurrent, leftSpeed);
    commandedRightSpeed = ff.calculate(rightSpeed) + pid.calculate(rightSpeedCurrent, rightSpeed);

    m_leftMotor.set(commandedLeftSpeed);
    m_rightMotor.set(commandedRightSpeed);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
