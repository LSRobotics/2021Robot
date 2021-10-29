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

import java.util.ArrayList;
import edu.wpi.first.wpilibj.PIDOutput;
import main.java.frc.robot.GyroPIDController;
import javax.lang.model.util.ElementScanner6;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PIDController;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Robot extends TimedRobot implements PIDOutput {

  public int ballsPresent;
  public double startTime;
  public boolean escape;
  public int autonStage;
  public static double TargetAngle;
  public static double gyroAngle;
  public double leftSpeed;
  public double rightSpeed;
  public double speedModifier;

  public GyroPIDController gyroPIDController;
  public PIDController straightPIDController;
  
  WPI_TalonFX front_left;
  WPI_TalonFX front_right;
  WPI_TalonFX back_left;
  WPI_TalonFX back_right;
  WPI_TalonFX climbMotor;
  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;
  DifferentialDrive driveMode;

  public WPI_TalonFX shooter;
  public VictorSPX intakeTop;
  public VictorSPX intakeBottom;
  public VictorSPX intakeToShooter;
  public VictorSPX intakeFront;
  public XboxController gp;
  public PowerDistributionPanel pdp;

  public Timer timer;

  //public PIDController smartPID;

  DigitalInput maxLimitSwitch;
  DigitalInput minLimitSwitch;

  public Compressor mCompressor;

  public DoubleSolenoid pneumatic_intake;
  public DoubleSolenoid pneumatic_ratchet;

  double kP = 0.03f;
  double kI = 0.00f;
  double kD = 0.00f;
  double kF = 0.00f;

  //sensors
  AnalogInput ultrasonic;
  AnalogInput IR;
  AHRS navx;

  boolean overrideClimb = false;

  boolean climbLock = true;

  @Override
  public void robotInit() {
    gp = new XboxController(0);

    ballsPresent = 0;
    autonStage = 0;
    TargetAngle = -1;
    leftSpeed = 0;
    rightSpeed = 0;
    speedModifier = Statics.lowModeModifier;
    shooter = new WPI_TalonFX(Statics.shooter);
    climbMotor = new WPI_TalonFX(Statics.climb_motor_id);
    intakeTop = new VictorSPX(Statics.intake_top);
    intakeBottom = new VictorSPX(Statics.intake_bottom);
    intakeToShooter = new VictorSPX(Statics.intake_toShooter);
    intakeFront = new VictorSPX(Statics.intake_front);

    timer = new Timer();

    pdp = new PowerDistributionPanel(0);
    
    front_left = new WPI_TalonFX(Statics.Wheel_FrontLeft);
    back_left = new WPI_TalonFX(Statics.Wheel_BackLeft);
    front_right = new WPI_TalonFX(Statics.Wheel_FrontRight);
    back_right = new WPI_TalonFX(Statics.Wheel_BackRight);

    mCompressor = new Compressor(0);
    
    maxLimitSwitch = new DigitalInput(0); //TODO move to static
    minLimitSwitch = new DigitalInput(1);

    pneumatic_intake = new DoubleSolenoid(Statics.pneumatic_intake_forward_channel, Statics.pneumatic_intake_backward_channel);
    pneumatic_ratchet = new DoubleSolenoid(Statics.pneumatic_climb_ratchet_forward_channel, Statics.pneumatic_climb_ratchet_backward_channel);
    IR = new AnalogInput(Statics.front_ir);

    pneumatic_intake.set(DoubleSolenoid.Value.kReverse);
    pneumatic_ratchet.set(DoubleSolenoid.Value.kForward);

    shooter.configFactoryDefault();
    front_left.configFactoryDefault();
    back_left.configFactoryDefault();
    front_right.configFactoryDefault();
    back_right.configFactoryDefault();

    leftMotors = new SpeedControllerGroup(front_left, back_left);
    rightMotors = new SpeedControllerGroup(front_right, back_right);

    driveMode = new DifferentialDrive(leftMotors, rightMotors);

    intakeBottom.follow(intakeTop);
    
    ultrasonic = new AnalogInput(Statics.ultrasonic);
    navx = new AHRS(); 
    gyroPIDController = new GyroPIDController(Statics.GYRO_P, Statics.GYRO_I, Statics.GYRO_D, Statics.GYRO_F, navx, new gyroPIDOutput());
    initializePIDControllers();

    climbMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void robotPeriodic() {

  }


  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    move(gp.getY(Hand.kLeft), gp.getX(Hand.kRight));
    intake(Statics.intakeSpeed * toInt(gp.getAButton()));
    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));
    if(gp.getXButtonPressed()) togglePneumaticIntake();
    climb(DPadToInt(gp.getPOV()), gp.getBumperPressed(Hand.kRight));
    checkSpeedToggle();
    //diagnostics();
   
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {

  }

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

  public void checkSpeedToggle()
  {
    if(gp.getBumperPressed(Hand.kLeft))
    {
      if(speedModifier == Statics.highModeModifier)
      {
        speedModifier = Statics.lowModeModifier;
      }
      else if(speedModifier == Statics.lowModeModifier)
      {
        speedModifier = Statics.highModeModifier;
      }
    }
  }
  public void test(){

    //gyroPIDController.setSetpoint(90);
    //updateMove();

  }
  

  public void initializePIDControllers() {
    gyroPIDController = new GyroPIDController(Statics.GYRO_P, Statics.GYRO_I, Statics.GYRO_D, Statics.GYRO_F, navx, new gyroPIDOutput());
    gyroPIDController.setPercentTolerance(Statics.PID_GYRO_TOLERANCE);
    gyroPIDController.enable();
    /*straightPIDController = new GyroPIDController(Statics.GYRO_P, Statics.GYRO_I, Statics.GYRO_D, Statics.GYRO_F, front_left, new gyroPIDOutput());
    straightPIDController.setPercentTolerance(Statics.PID_GYRO_TOLERANCE);
    straightPIDController.enable();*/
    
  }
  
  private void updateMove() {
    if(TargetAngle != -1) {
      if(gyroPIDController.onTarget()) {
        TargetAngle = -1;
        autonStage+= 1;
      }
      else {
        gyroPIDController.calculate();
        move(leftSpeed, rightSpeed);
      }
    }
    
  }

  public void rotateDegrees(double angle)
  {
    navx.reset();
    gyroPIDController.reset();
    gyroPIDController.setPID(kP, kI, kD);
    gyroPIDController.setSetpoint(angle);
    gyroPIDController.enable();
  }
  
  public void moveTo(double inches)
  {
    straightPIDController.setSetpoint(inches * Statics.inchToSensorUnits);

    /*if(front_left.getSelectedSensorPosition() <= inches * Statics.inchToSensorUnits)
    {
      front_left.set(ControlMode.PercentOutput, Statics.moveSpeed);
      front_right.set(ControlMode.PercentOutput, Statics.moveSpeed);
    }
    else
    {
      front_left.set(ControlMode.PercentOutput, 0);
      front_right.set(ControlMode.PercentOutput, 0);
    }*/
  }
  
  public void changeClimbLock(boolean state) {

    if (state && pneumatic_ratchet.get() != DoubleSolenoid.Value.kForward) { //is locked
      //lock the ratchet 
      pneumatic_ratchet.set(DoubleSolenoid.Value.kForward);
    }
    else if (!state && pneumatic_ratchet.get() != DoubleSolenoid.Value.kReverse){ //isn't locked
      //unlock the ratchet
      pneumatic_ratchet.set(DoubleSolenoid.Value.kReverse);
    }

  }
  
  public void move(double throttle, double turn) {
    if(Math.abs(throttle) <= Statics.stickDeadzone){
      throttle = 0;
    }

    if (Math.abs(turn) <= Statics.stickDeadzone) {
      turn = 0;
    }

    //throttle *= Math.abs(throttle); just case we want to square it
    //turn *= Math.abs(turn);

    driveMode.arcadeDrive(throttle * speedModifier, turn * speedModifier);

  }

  public void shoot(double speed) {
    shooter.set(ControlMode.PercentOutput, speed);
  }

  /*public void indexClear(double speed){
    ballsPresent = 0;
    shooter.set(ControlMode.PercentOutput, speed);
    
    //intakeBottom.set(ControlMode.PercentOutput, -speed);
    intakeToShooter.set(ControlMode.PercentOutput, -speed);
  }*/

  public int DPadToInt(int angle)
  {
    if (angle == -1) return -1;
    return angle / 45;
    
  }
  
  public void climb(int direction, boolean lockInput)
  {
    if (lockInput) {climbLock = !climbLock;}

    if(direction == 0 && !maxLimitSwitch.get()) //going up!
    {
      climbLock = true;
      climbMotor.set(-0.5); //TODO change to static
    }
    else if(direction == 4 && !minLimitSwitch.get()) //going down :(
    {
      climbLock = false;
      climbMotor.set(0.5);
    }
    else
    {
      climbMotor.set(0);
    }

    changeClimbLock(climbLock);

  }

  public void intake(double speed) {
    double irVoltage = IR.getVoltage();
    if(irVoltage > Statics.intakeIRThreshold)
    {
      if(!escape)
      {
        startTime = timer.get();
        escape = true;
      }
      intakeFront.set(ControlMode.PercentOutput, -speed * 1.25);
    }
    if(irVoltage > Statics.intakeIRThreshold && (timer.get() - startTime) > 2)
    {
      intakeFront.set(ControlMode.PercentOutput, 0);
      escape = false;
    }
    else if(irVoltage < Statics.intakeIRThreshold)
    {
      intakeFront.set(ControlMode.PercentOutput, -speed * 1.25);
      startTime = 0;
    }
     
  }

  

  public double setToShooterBeltSpeed()
  {
    double speed = 0;
    if(ballsPresent == 3)
    {
        speed = Statics.intakeSpeed * toInt(gp.getAButton());
    }
    /*if(startTime != 0)
      {
        speed = -0.4;
      }*/
      //speed = Statics.intakeSpeed * toInt(gp.getAButton());
  if(shooter.getSelectedSensorVelocity() <= Statics.shooterVelocity)
      {
        ballsPresent = 0;
        speed = Statics.intakeSpeed;
      }
      if(toInt(gp.getYButton()) != 0)
      {
      speed = -(Statics.intakeSpeed * toInt(gp.getYButton()));
      }
    return speed;
  }

  public double setBeltSpeed()
  {
    boolean escape = false;
    double irVoltage = IR.getVoltage();
    double speed = 0;
    if (irVoltage > Statics.intakeIRThreshold && toInt(gp.getYButton()) == 0)
    {
      escape = true;
      switch(ballsPresent)
      {
        case 0:
        speed = -.3;
        break;
        case 1:
        speed = -.3;
        //startTime = timer.get();
        break;
        case 2:
        speed = -.7;
        break;
        case 3:
        speed = -1;
        break;
        case 4:
        speed = -1;
        break;
        case 5:
        speed = -1;
        break;
        
      }
      
      return speed;
      //intakeBottom.set(ControlMode.PercentOutput, -speed * toInt(gp.getAButton())); 
      //intakeTop.set(ControlMode.PercentOutput, -speed * toInt(gp.getAButton()));
      
      
      
    }
    else if(irVoltage <= Statics.intakeIRThreshold && escape)
    {
      escape = false;
      ballsPresent++;
    }
   /*
    if(startTime != 0)
      {
        intakeToShooter.set(ControlMode.PercentOutput, -0.4);
        //intakeBottom.set(ControlMode.PercentOutput, -0.4); 
        speed = -0.4;
      }
      else if((timer.get() - startTime) > 3)
      {
        startTime = 0;
        
      }*/
      if(shooter.getSelectedSensorVelocity() <= Statics.shooterVelocity)
      {
        //BALLS ZERO IN BELTSHOOTER
        speed = -Statics.intakeSpeed;
      }
      if(toInt(gp.getYButton()) != 0)
      {
      speed = (Statics.intakeSpeed * toInt(gp.getYButton()));
      }
      
    return speed;
  }

  public int toInt(boolean condition) {
    return condition ? 1 : 0;
  }

  //function to convert voltage recieved from maxbotix ultrasonic sensor to a distance in inches
  //example: getRangeInches(maxbotixFront_US.getValue());
  public double getRangeInches(double rawVoltage){
    return rawVoltage * Statics.cm_to_in;
  }

  public void pidWrite(double output) {
    leftSpeed = output;
    rightSpeed = output;
  }
  public void diagnostics() {

    SmartDashboard.putNumber("Drive Motor Left", front_right.get());
    SmartDashboard.putNumber("Drive Motor Right", front_left.get());

    SmartDashboard.putNumber("Intake Front", intakeFront.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Voltage", pdp.getVoltage());

    SmartDashboard.putNumber("Ultrasonic Distance", getRangeInches(ultrasonic.getValue()));
    
    SmartDashboard.putBoolean("Compressor Running", mCompressor.getPressureSwitchValue());
    SmartDashboard.putBoolean("Compressor is Enabled", mCompressor.enabled());

    SmartDashboard.putNumber("Direction", DPadToInt(gp.getPOV()));

    SmartDashboard.putNumber("Intake Sensor", IR.getVoltage());
    SmartDashboard.putNumber("Balls Present", ballsPresent);
    SmartDashboard.putNumber("NAVX Z-Axis", navx.getYaw());
  }

  private class gyroPIDOutput implements PIDOutput {

    public void pidWrite(double output) {
      leftSpeed = output;
      rightSpeed = -output; 
    }
  }

  
}

