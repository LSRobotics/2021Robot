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
import com.kauailabs.navx.frc.AHRS;

import java.util.ArrayList;

import javax.lang.model.util.ElementScanner6;

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
import com.kauailabs.navx.frc.AHRS;



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
  WPI_TalonFX climb;
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
  public AHRS navx;

  //public PIDController smartPID;
  

  DigitalInput limitSwitch;


  public static Compressor mCompressor;

  //public static DoubleSolenoid pneumatic_intake;
  //public static DoubleSolenoid pneumatic_ratchet;
 // public static DoubleSolenoid pneumatic_drive_train_gear_shift;

  static double kP = 0.03f;
  static double kI = 0.00f;
  static double kD = 0.00f;
  static double kF = 0.00f;

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
    climb = new WPI_TalonFX(11);
    intakeTop = new VictorSPX(Statics.intake_top);
    intakeBottom = new VictorSPX(Statics.intake_bottom);
    intakeToShooter = new VictorSPX(Statics.intake_toShooter);
    intakeFront = new VictorSPX(Statics.intake_front);


    pdp = new PowerDistributionPanel(0);
    
    front_left = new WPI_TalonFX(Statics.Wheel_FrontLeft);
    back_left = new WPI_TalonFX(Statics.Wheel_BackLeft);
    front_right = new WPI_TalonFX(Statics.Wheel_FrontRight);
    back_right = new WPI_TalonFX(Statics.Wheel_BackRight);

    mCompressor = new Compressor(0);

    
    limitSwitch = new DigitalInput(1);


   // pneumatic_intake = new DoubleSolenoid(pneumatic_intake_forward_channel, pneumatic_intake_backward_channel);
    //pneumatic_ratchet = new DoubleSolenoid(pneumatic_climb_ratchet_forward_channel, pneumatic_climb_ratchet_backward_channel);
   // pneumatic_drive_train_gear_shift = new DoubleSolenoid(pneumatic_drive_train_gear_shift_forward_channel, pneumatic_drive_train_gear_shift_backward_channel);

    shooter.configFactoryDefault();
    front_left.configFactoryDefault();
    back_left.configFactoryDefault();
    front_right.configFactoryDefault();
    back_right.configFactoryDefault();

    back_right.follow(front_right);
    back_left.follow(front_left);
    
    maxbotixFront_US = new AnalogInput(Statics.US_Maxbotix_Front);
    navx = new AHRS(); 

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
      case TEST:
      test();
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

    /*
    //mCompressor.setClosedLoopControl(true);
    System.out.println("Pressure Switch Value: "+ mCompressor.getPressureSwitchValue());
    System.out.println("Current: "+ mCompressor.getCompressorCurrent());
    System.out.println("ClosedLoopCOntrol: "+ mCompressor.getClosedLoopControl());
    System.out.println("6: "+ mCompressor.getCompressorCurrentTooHighFault());
    System.out.println("5: "+ mCompressor.getCompressorCurrentTooHighStickyFault());
    System.out.println("4: "+ mCompressor.getCompressorNotConnectedFault());
    System.out.println("3: "+ mCompressor.getCompressorNotConnectedStickyFault());
    System.out.println("2: "+ mCompressor.getCompressorShortedFault());
    System.out.println("1: "+ mCompressor.getCompressorShortedStickyFault());
    mCompressor.start();
    */


    move(gp.getY(Hand.kLeft), gp.getY(Hand.kRight));

    //intake(Statics.intakeSpeed * (toInt(gp.getAButton()) - toInt(gp.getXButton())));
    intake(Statics.intakeSpeed * toInt(gp.getAButton()));

    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));
    
    //climb(DPadToInt(gp.getPOV()));

    diagnostics();

    /*if (gp.getXButtonPressed()) {
      switch (pneumatic_intake.get()) {
        case DoubleSolenoid.Value.kForward: pneumatic_intake.set(DoubleSolenoid.Value.kReverse);
        break;
        case DoubleSolenoid.Value.kReverse: pneumatic_intake.set(DoubleSolenoid.Value.kForward);
        break;
        case DoubleSolenoid.Value.kOff: pneumatic_intake.set(DoubleSolenoid.Value.kForward);
        break;
      }
    }*/

  }

  public void competition1Periodic(){
 // move(pid.calculate());
    
    

  }
  public void competition2Periodic(){
    
    //navx.getYaw();
  }
  public void competition3Periodic(){
    move(gp.getY(Hand.kLeft), gp.getY(Hand.kRight));
    /*if (gp.getXButton()) {
      pneumatic_intake.set(DoubleSolenoid.Value.kForward);
    }*/
    
    intake(Statics.intakeSpeed * toInt(gp.getAButton()));
    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));
    
  }
  public void competition4Periodic(){
    //PUT IN PIXY CODE
    
    move(gp.getY(Hand.kLeft), gp.getY(Hand.kRight));
    /*if (gp.getXButton()) {

      pneumatic1.set(DoubleSolenoid.Value.kForward);

    }
    else if (gp.getYButton()) {
      pneumatic1.set(DoubleSolenoid.Value.kReverse);
    }*/

    intake(Statics.intakeSpeed * toInt(gp.getAButton()));
    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));



  }
  public void competition5Periodic(){
    
  }
  public void test(){
    System.out.println(front_left.getSelectedSensorPosition());
    System.out.println(back_left.getSelectedSensorPosition());
    System.out.println(front_right.getSelectedSensorPosition());
    System.out.println(front_right.getSelectedSensorPosition());
    //MULTIPLY BY FRACTION THING? CALCULATE METERS TRAVELLED PER UNITS 

  }
  

  public void setSetpoint(int setpoint)
  {

  }
  public void PID()
  {

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

  
    shooter.set(-speed);


  }

  public int DPadToInt(int angle)
  {
    return angle / 45;
    
  }
  public void climb(int direction)
  {

    if(direction == 0) //going up!
    {
      climb.set(-0.1);
    }
    else if(direction == 4) //going down :(
    {
      climb.set(0.1);
    }
   /* else if(null)
    {
      climb.set(0);
    }*/
    else
    {
      climb.set(0);
    }
    

   SmartDashboard.putNumber("Direction", DPadToInt(gp.getPOV()));
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
