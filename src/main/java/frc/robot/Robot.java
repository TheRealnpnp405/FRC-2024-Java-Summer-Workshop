// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // motor declarations
  public static final CANSparkMax m_leftMotorLEAD = new CANSparkMax(1, MotorType.kBrushless);
  public static final CANSparkMax m_leftMotorFOLLOW = new CANSparkMax(2, MotorType.kBrushless);
  public static final CANSparkMax m_rightMotorLEAD = new CANSparkMax(3, MotorType.kBrushless);
  public static final CANSparkMax m_RightMotorFOLLOW = new CANSparkMax(4, MotorType.kBrushless);
  public static final CANSparkMax m_Feeder = new CANSparkMax(5, MotorType.kBrushless);
  public static final CANSparkMax m_Shooter = new CANSparkMax(6, MotorType.kBrushless);
  public static final CANSparkMax m_AngleLEAD = new CANSparkMax(7, MotorType.kBrushless);
  public static final CANSparkMax m_AngleFOLLOW = new CANSparkMax(8, MotorType.kBrushless);
  public static final SparkAbsoluteEncoder.Type kDutyCycle = Type.kDutyCycle;
  // Drive Control
  private static final DifferentialDrive Drive_Main = new DifferentialDrive(m_leftMotorLEAD, m_rightMotorLEAD);
  private double speedMultiplier = 0;
  private static double xCorrect = .65; // used to smooth out turning
  private boolean forwardDriveToggle = true;

  // Shooting Var
  private static double intakeSpeed = 0.2;
  private static double shootSpeed = -1;
  private double raw_speed;
  private double currentDif;
  // tilt values //TODO find out what these values are for you!
  protected final double maxAngle = 0.1727; // Can't be less than
  protected final double minAngle = 0.6425; // Can't be greater than
  private final double shooterTiltSpeaker = 0.6488;
  private final double shooterTiltAmp = 0.1599;

  // Different auto angle speeds
  private final double normalAngleSpeed = 0.375;
  private final double normalAngleSlope = 125;
  private final double hangAngleSpeed = 0.4;
  private final double hangAngleSlope = 90;

  // Autonomous
  private int autoStep;

  // Cameras
  UsbCamera USBCamera;
  UsbCamera USBCamera1;
  NetworkTableEntry cameraSelection;

  // Joystick
  private static final Joystick flight = new Joystick(0);
  // Flight Buttons
  private static final int flight1 = 1; // Trigger
  private static final int flight2 = 2; // Thumb
  // private static final int flight3 = 3;
  private static final int flight4 = 4;
  // private static final int flight5 = 5;
  // private static final int flight6 = 6;
  // private static final int flight7 = 7;
  // private static final int flight8 = 8;
  // private static final int flight9 = 9;
  // private static final int flight10 = 10;
  // private static final int flight11 = 11;
  // private static final int flight12 = 12;
  // Flight Other
  private static final int flightPaddle = 3;
  // Auto Drive
  private static double EncoderPos_to_Feet = 5.380945682525635; // TODO Find encoder conversion constant.
  private static final String kRight = "Turn Right";
  private static final String kLeft = "Turn Left";

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Drive motors init
    m_RightMotorFOLLOW.follow(m_rightMotorLEAD);
    m_leftMotorFOLLOW.follow(m_leftMotorLEAD);
    m_leftMotorLEAD.setInverted(true);
    m_rightMotorLEAD.setInverted(false);
    Drive_Main.setSafetyEnabled(false);
    m_AngleFOLLOW.follow(m_AngleLEAD, true);
    // Camera Server enable
    USBCamera = CameraServer.startAutomaticCapture(0);
    USBCamera1 = CameraServer.startAutomaticCapture(1);
    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("cameraSelection");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // ********************************
    // * SMARTDASHBOARD
    // ********************************

    // Motor and control info
    SmartDashboard.putNumber("Encoder Right", m_rightMotorLEAD.getEncoder().getPosition());
    SmartDashboard.putNumber("Encoder Left", m_leftMotorLEAD.getEncoder().getPosition());
    SmartDashboard.putNumber("Current Difference", currentDif);
    SmartDashboard.putNumber("Angle Speed", raw_speed);
    SmartDashboard.putNumber("Angle", m_AngleLEAD.getAbsoluteEncoder(kDutyCycle).getPosition());
    SmartDashboard.putNumber("Lime Distance", getLimeZ());

    // Drive Helpers
    SmartDashboard.putNumber("speedMultiplier", speedMultiplier);
    SmartDashboard.putBoolean("Forward Drive", forwardDriveToggle);
    SmartDashboard.putNumber("Y", flight.getY());
    SmartDashboard.putNumber("X", flight.getX());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_rightMotorLEAD.getEncoder().setPosition(0);
    m_leftMotorLEAD.getEncoder().setPosition(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // ********************************
    // * DRIVE CODE
    // ********************************
    // Drive with arcade drive. Y axis drives forward and backward, and the X turns
    // left and right.
    // Shooter side is the front of the robot.
    speedMultiplier = (((flight.getRawAxis(flightPaddle) * -1) * .2) + .6);
    Drive_Main.arcadeDrive(flight.getY() * speedMultiplier, flight.getX() * xCorrect);

    // Flip logic
    if (flight.getRawButton(flight2)) {
      if (forwardDriveToggle) {
        forwardDriveToggle = false;
        m_leftMotorLEAD.setInverted(false);
        m_rightMotorLEAD.setInverted(true);
        xCorrect = Math.abs(xCorrect) * -1;
      } else {
        forwardDriveToggle = true;
        m_leftMotorLEAD.setInverted(true);
        m_rightMotorLEAD.setInverted(false);
        xCorrect = Math.abs(xCorrect);
      }
      // prevent doubleclick of the flip button
      try {
        Thread.sleep(300);
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }

    // ********************************
    // * SHOOT CODE FOR FLIGHT STICK
    // ********************************
    // LIMELIGHT REF
    // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
    if (flight.getRawButton(flight1)) {
      m_Shooter.set(shootSpeed);
      wait(500);
      m_Feeder.set(shootSpeed);
    } else if (flight.getRawButton(flight4)) {
      m_Shooter.set(intakeSpeed);
      m_Feeder.set(intakeSpeed);
    } else {
      m_Shooter.stopMotor();
      m_Feeder.stopMotor();
    }

    // ********************************
    // * ANGLE CODE FOR FLIGHT STICK
    // ********************************
    // if i press 7 or 5, go to shooterTiltSpeaker, normalAngleSpeed,
    // normalAngleSlope
    // if i press 8 or 3, go to shooterTiltAmp, normalAngleSpeed, normalAngleSlope
    // else don't move the motor
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    m_rightMotorLEAD.getEncoder().setPosition(0);
    m_leftMotorLEAD.getEncoder().setPosition(0);
    autoStep = 0;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (autoStep > 1) {
      driveStraightWithEncoder(0.4, 2);
    } else if (autoStep < 2) {
      ArcLengthTurnWithEncoder(.4, 2, kRight);
    } else if (autoStep < 3) {
      driveStraightWithEncoder(0.4, 2);
    } else if (autoStep < 4) {
      ArcLengthTurnWithEncoder(.4, 2, kLeft);
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  // ********************************
  // * CUSTOM FUNCTIONS
  // ********************************

  /**
   * This function creates a wait
   * 
   * @param time = Time to wait in milliseconds.
   */
  public static void wait(int time) {
    try {
      Thread.sleep(time);
    } catch (InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
  }

  /**
   * Gets the current Limelight Z distance.
   */
  public static double getLimeZ() {
    var calcs = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    return calcs.getZ();
  }

  /**
   * This function controls the tilt of the m_Angle motor.
   * 
   * @param DesiredPos  = Position m_Angle should be set to.
   * @param Angle_Speed = Angle speed can be "normalAngleSpeed" or
   *                    "hangAngleSpeed."
   * @param Slope       = Slope to be used with equation.
   */
  public void AngleControl(double DesiredPos, double Angle_Speed, double Slope) {
    double currentPos = m_AngleLEAD.getAbsoluteEncoder(kDutyCycle).getPosition();
    currentDif = (DesiredPos - currentPos);
    if (DesiredPos == currentPos) {
      m_AngleLEAD.stopMotor();
    } else {
      if (currentDif >= 0) {
        raw_speed = (Slope * (Math.pow(currentDif, 2))); // Initial speed calculation
        if (raw_speed > Angle_Speed) { // Check to ensure speed is not exceeding 100%
          m_AngleLEAD.set(Angle_Speed);
        } else {
          m_AngleLEAD.set(raw_speed);
        }
      } else {
        raw_speed = (Slope * (Math.pow(currentDif, 2))); // Initial speed calculation
        if (raw_speed > Angle_Speed) { // Check to ensure speed is not exceeding 100%
          m_AngleLEAD.set(-1 * Angle_Speed);
        } else {
          m_AngleLEAD.set(-1 * raw_speed);
        }
      }
    }
  }

  /**
   * Drive straight with encoders
   * 
   * @param power    = Speed to drive the drivetrain
   *                 (negative drives lime side, positive drives battery side)
   * @param distance = How far to drive in feet
   */
  public void driveStraightWithEncoder(double power, double distance) {
    double error = Math.abs(m_leftMotorLEAD.getEncoder().getPosition()
        - m_rightMotorLEAD.getEncoder().getPosition()); // Difference between the encoder on the left and rights sides.
    double kP = 0.05; // Proportional control for driving straight.
    double feet = Math
        .abs(((m_rightMotorLEAD.getEncoder().getPosition() + m_leftMotorLEAD.getEncoder().getPosition()) / 2)
            / EncoderPos_to_Feet);
    if (Math.abs(feet) < distance) {
      Drive_Main.tankDrive((power + (kP * error)), (power + (kP * error)));
    } else {
      Drive_Main.stopMotor();
      autoStep++;
      System.out.println("STEP " + autoStep + " DONE");
      m_rightMotorLEAD.getEncoder().setPosition(0);
      m_leftMotorLEAD.getEncoder().setPosition(0);
    }
  }

  /**
   * Turn based on arc length
   * 
   * @param power      = Speed to drive
   * @param arc_length = How far to drive in feet
   * @param direction  = Turning direction, "true" left or "false" right
   */
  public void ArcLengthTurnWithEncoder(double power, double arc_length, String direction) {
    double error = 0;
    double kP = 1; // Proportional control for driving straight
    double Lfeet = m_leftMotorLEAD.getEncoder().getPosition() / EncoderPos_to_Feet;
    double Rfeet = m_rightMotorLEAD.getEncoder().getPosition() / EncoderPos_to_Feet;

    if (direction == kRight && Math.abs(Rfeet) < arc_length) {
      Drive_Main.tankDrive(0, power + (kP * error));
    } else if (direction == kLeft && Math.abs(Lfeet) < arc_length) {
      Drive_Main.tankDrive(power + (kP * error), 0);
    } else {
      Drive_Main.stopMotor();
      autoStep++;
      System.out.println("STEP " + autoStep + " DONE");
      m_rightMotorLEAD.getEncoder().setPosition(0);
      m_leftMotorLEAD.getEncoder().setPosition(0);
    }
  }
}