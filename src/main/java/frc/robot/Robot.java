/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants.Statics;
import frc.robot.Constants.Statics.CompetitionSelection;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import edu.wpi.first.wpilibj.DigitalInput;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  WPI_TalonFX front_left;
  WPI_TalonFX front_right;
  WPI_TalonFX back_left;
  WPI_TalonFX back_right;
  DifferentialDrive drive;
  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;
  public WPI_TalonFX shooter;
  public VictorSPX intakeTop;
  public VictorSPX intakeBottom;
  public VictorSPX intakeToShooter;
  public VictorSPX intakeFront;
  public XboxController gp;
  public PowerDistributionPanel pdp;
  public 
  

  DigitalInput limitSwitch;


  public static Compressor mCompressor;

  public static DoubleSolenoid pneumatic1;
  public static DoubleSolenoid pneumatic2;
  public static DoubleSolenoid pneumatic3;

  //sensors
  AnalogInput maxbotixFront_US;




  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    gp = new XboxController(0);

    shooter = new WPI_TalonFX(Statics.shooter);
    intakeTop = new VictorSPX(Statics.intake_top);
    intakeBottom = new VictorSPX(Statics.intake_bottom);
    intakeToShooter = new VictorSPX(Statics.intake_toShooter);
    intakeFront = new VictorSPX(Statics.intake_front);

    pdp = new PowerDistributionPanel(0);
  
    pdp.clearStickyFaults();
    
    front_left = new WPI_TalonFX(Statics.Wheel_FrontLeft);
    back_left = new WPI_TalonFX(Statics.Wheel_BackLeft);
    front_right = new WPI_TalonFX(Statics.Wheel_FrontRight);
    back_right = new WPI_TalonFX(Statics.Wheel_BackRight);

    mCompressor = new Compressor(0);

    
    limitSwitch = new DigitalInput(1);


    pneumatic1 = new DoubleSolenoid(Statics.pneumatic1_forward_channel, Statics.pneumatic1_back_channel);
    pneumatic2 = new DoubleSolenoid(Statics.pneumatic2_forward_channel, Statics.pneumatic2_back_channel);
    pneumatic3 = new DoubleSolenoid(Statics.pneumatic3_forward_channel, Statics.pneumatic3_back_channel);

    shooter.configFactoryDefault();
    front_left.configFactoryDefault();
    back_left.configFactoryDefault();
    front_right.configFactoryDefault();
    back_right.configFactoryDefault();

    back_right.follow(front_right);
    back_left.follow(front_left);
    
    maxbotixFront_US = new AnalogInput(Statics.US_Maxbotix_Front);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {


    //Determine which competition is being run by value in statics
    switch(Statics.current_competition)
    {
      case COMPETITION_1:
      competition1Periodic();
      break;
      case COMPETITION_2:
      competition2Periodic();
      break;
      case COMPETITION_3:
      competition3Periodic();
      break;
      case COMPETITION_4:
      competition4Periodic();
      break;
      case COMPETITION_5:
      competition5Periodic();
      break;

    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
   

  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    move(gp.getY(Hand.kLeft), gp.getY(Hand.kRight));

    //intake(Statics.intakeSpeed * (toInt(gp.getAButton()) - toInt(gp.getXButton())));
    intake(Statics.intakeSpeed * toInt(gp.getAButton()));

    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));
    
    diagnostics();

    if (gp.getXButton()) {

      pneumatic1.set(DoubleSolenoid.Value.kForward);

    }
    else if (gp.getYButton()) {
      pneumatic1.set(DoubleSolenoid.Value.kReverse);
    }

  }

  public void competition1Periodic(){

  }
  public void competition2Periodic(){
    
  }
  public void competition3Periodic(){
    
  }
  public void competition4Periodic(){
    
  }
  public void competition5Periodic(){
    
  }



  public void move(double leftThrottle, double rightThrottle) {


    //instead of checking for both the positive and the negative versions, just take the absolute value so you only have to check once
    if(Math.abs(rightThrottle) >= Statics.stickDeadzone){
      front_right.set(ControlMode.PercentOutput, rightThrottle);
    }
    else {
      front_right.set(ControlMode.PercentOutput, 0);
    }

    if(Math.abs(leftThrottle) >= Statics.stickDeadzone){
      front_left.set(ControlMode.PercentOutput, -leftThrottle);
    }
    else {
      front_left.set(ControlMode.PercentOutput, 0);
    }


  }

  public void shoot(double speed) {

  
    shooter.set(speed);


  }

  public void intake(double speed) {

    intakeTop.set(ControlMode.PercentOutput, -speed);
    intakeBottom.set(ControlMode.PercentOutput, -speed);
    intakeToShooter.set(ControlMode.PercentOutput, speed);
    intakeFront.set(ControlMode.PercentOutput, -speed);

  }

  public int toInt(boolean condition) {
    return condition ? 1 : 0;
  }

  

  //function to convert voltage recieved from maxbotix ultrasonic sensor to a distance in inches
  //example: getRangeInches(maxbotixFront_US.getValue());
  public double getRangeInches(double rawVoltage){
    return rawVoltage * Statics.cm_to_in;
  }

  public void diagnostics() {

    SmartDashboard.putNumber("Drive Motor Left", front_right.get());
    SmartDashboard.putNumber("Drive Motor Right", front_left.get());

    SmartDashboard.putNumber("Intake Front", intakeFront.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Voltage", pdp.getVoltage());

    SmartDashboard.putNumber("Ultrasonic Distance", getRangeInches(maxbotixFront_US.getValue()));
    
    SmartDashboard.putBoolean("Compressor Running", mCompressor.getPressureSwitchValue());
    SmartDashboard.putBoolean("Compressor is Enabled", mCompressor.enabled());
  }


}
