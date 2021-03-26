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
  

  //public PIDController smartPID;
  

  DigitalInput limitSwitch;


  public static Compressor mCompressor;

  public static DoubleSolenoid pneumatic_intake;
  public static DoubleSolenoid pneumatic_ratchet;
  public static DoubleSolenoid pneumatic_drive_train_gear_shift;

  static double kP = 0.03f;
  static double kI = 0.00f;
  static double kD = 0.00f;
  static double kF = 0.00f;

  //sensors
  AnalogInput maxbotixFront_US;
  AnalogInput IR;




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


    pneumatic_intake = new DoubleSolenoid(Statics.pneumatic_intake_forward_channel, Statics.pneumatic_intake_backward_channel);
    pneumatic_ratchet = new DoubleSolenoid(Statics.pneumatic_climb_ratchet_forward_channel, Statics.pneumatic_climb_ratchet_backward_channel);
    pneumatic_drive_train_gear_shift = new DoubleSolenoid(Statics.pneumatic_drive_train_gear_shift_forward_channel, Statics.pneumatic_drive_train_gear_shift_backward_channel);
    IR = new AnalogInput(Statics.front_ir);

    pneumatic_intake.set(DoubleSolenoid.Value.kReverse);
    pneumatic_ratchet.set(DoubleSolenoid.Value.kReverse);
    pneumatic_drive_train_gear_shift.set(DoubleSolenoid.Value.kReverse);

    shooter.configFactoryDefault();
    front_left.configFactoryDefault();
    back_left.configFactoryDefault();
    front_right.configFactoryDefault();
    back_right.configFactoryDefault();

    back_right.follow(front_right);
    back_left.follow(front_left);
    
    maxbotixFront_US = new AnalogInput(Statics.US_Maxbotix_Front);
    //navx = new AHRS(); 

  }

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
    }
  
  public void togglePneumaticIntake()
  {
    
      switch (pneumatic_intake.get()) {
        case kForward: pneumatic_intake.set(DoubleSolenoid.Value.kReverse);
        break;
        case kReverse: pneumatic_intake.set(DoubleSolenoid.Value.kForward);
        break;
        case kOff: pneumatic_intake.set(DoubleSolenoid.Value.kForward);
        break;
      }
    
  }

  public void competition1Periodic(){
 // move(pid.calculate());
    
    

  }
  public void competition2Periodic(){
    
    switch(Statics.current_part){
    case PART_1:
    
      //Forward 150 in
      //Rotate 90
      //Forward 48 in
      //Rotate 180
      //Forward 48 in
      //Rotate 270
      //Forward 48 in
      //Rotate to 0
      //Forward 168 in
      //Rotate 270
      //Forward 48 in
      //Rotate 180
      //Forward 48 in
      //Rotate 90
      //Forward 96 in
      //Rotate 0
      //Forward 96 in
      //Rotate 270
      //Forward 48
      //Rotate 180
      //Forward 250 in
    break;
    case PART_2:
      //CCW 30"
      //CW 90"
      //CCW 30"
      //CW 90"
      //CCW 30"
    break;
    case PART_3:
      //CCW - Forward - 60"
      //
    break;
  }
    //navx.getYaw();
  }
  public void competition3Periodic(){
    move(gp.getY(Hand.kLeft), gp.getY(Hand.kRight));
    if (gp.getXButton()) {
      pneumatic_intake.set(DoubleSolenoid.Value.kForward);
    }
    
    intake(Statics.intakeSpeed * toInt(gp.getAButton()));
    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));
    
  }
  public void competition4Periodic(){
    
    
    move(gp.getY(Hand.kLeft), gp.getY(Hand.kRight));
    

    intake(Statics.intakeSpeed * toInt(gp.getAButton()));
    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));



  }
  public void competition5Periodic(){
    
  }
  public void test(){
    move(gp.getY(Hand.kLeft), gp.getY(Hand.kRight));
    intake(Statics.intakeSpeed * toInt(gp.getAButton()));
    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));
    //indexClear(Statics.indexClearSpeed * toInt(gp.getXButton()));
    if(gp.getXButtonPressed())
    {
      togglePneumaticIntake();
    }
    detectBall();
   
    

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
    //intakeToShooter.set(ControlMode.PercentOutput, speed);
    if(shooter.getSelectedSensorVelocity() <= Statics.shooterVelocity)
    {
      intakeToShooter.set(ControlMode.PercentOutput, Statics.intakeSpeed);
    }
    shooter.set(ControlMode.PercentOutput, speed);
    System.out.println("Shooter speed: " + shooter.getSelectedSensorVelocity());
  }

  public void indexClear(double speed){
    shooter.set(ControlMode.PercentOutput, -speed);
    intakeTop.set(ControlMode.PercentOutput, -speed);
    intakeBottom.set(ControlMode.PercentOutput, -speed);
    intakeToShooter.set(ControlMode.PercentOutput, speed);
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

  public void detectBall()
  {
    double irVoltage = IR.getAverageVoltage();
    if(irVoltage > 1.5)
    {
      //intakeTop.set(ControlMode.PercentOutput, Statics.intakeSpeed);
      //intakeBottom.set(ControlMode.PercentOutput, Statics.intakeSpeed);
      //intakeToShooter.set(ControlMode.PercentOutput, Statics.intakeSpeed);
    }
    System.out.println("VOLTAGE IR:" + irVoltage);
    
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
