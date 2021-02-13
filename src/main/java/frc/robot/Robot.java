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
  TalonFX front_left;
  TalonFX front_right;
  TalonFX back_left;
  TalonFX back_right;
  DifferentialDrive drive;
  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;
  public WPI_TalonFX shooter;
  public VictorSPX vic5;
  public VictorSPX vic6;
  public VictorSPX vic7;
  public VictorSPX vic8;
  public XboxController gp;




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

    shooter = new WPI_TalonFX(9);
    vic5 = new VictorSPX(5);
    vic6 = new VictorSPX(6);
    vic7 = new VictorSPX(7);
    vic8 = new VictorSPX(8);

    
    front_left = new TalonFX(Statics.Wheel_FrontLeft);
    back_left = new TalonFX(Statics.Wheel_BackLeft);
    front_right = new TalonFX(Statics.Wheel_FrontRight);
    back_right = new TalonFX(Statics.Wheel_BackRight);

    back_right.follow(front_right);
    back_left.follow(front_left);
    


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

    intake(Statics.intakeSpeed * toInt(gp.getAButton()));

    shoot(-Statics.shooterSpeed * toInt(gp.getBButton()));

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

    vic5.set(ControlMode.PercentOutput, -speed);
    vic6.set(ControlMode.PercentOutput, -speed);
    vic7.set(ControlMode.PercentOutput, speed);
    vic8.set(ControlMode.PercentOutput, -speed);

  }

  public int toInt(boolean condition) {
    return condition ? 1 : 0;
  }

}
