/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//TODO Change to Arcade Drive
//TODO Remove Auton
//TODO Remove Gear Shift
//TODO Updating assist features
//TODO add Demo Mode

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

public class Robot extends TimedRobot implements PIDOutput {

  public int ballsPresent = 0;
  public double startTime;
  public boolean escape;
  public int autonStage = 0;
  public static double TargetAngle = -1;
  public static double gyroAngle;
  public double leftSpeed = 0;
  public double rightSpeed = 0;
  public double speedModifier = Statics.lowModeModifier;

  public GyroPIDController gyroPIDController;
  public PIDController straightPIDController;

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  WPI_TalonFX driveFrontleft;
  WPI_TalonFX driveFrontright;
  WPI_TalonFX driveBackleft;
  WPI_TalonFX driveBackright;
  WPI_TalonFX climb;
  DifferentialDrive drive;
  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;
  public WPI_TalonFX shooter;
  public VictorSPX intakeTop;
  public VictorSPX intakeBottom;
  public VictorSPX intakeToshooter;
  public VictorSPX intakeFront;

  public PowerDistributionPanel pdp;
  public static Compressor compressor;

  public Timer timer;

  //public PIDController smartPID;

  public static DoubleSolenoid pneumaticIntake;
  public static DoubleSolenoid pneumaticRatchet;
  public static DoubleSolenoid pneumaticDrivetrainGearshift;

  static double kP = 0.03f;
  static double kI = 0.00f;
  static double kD = 0.00f;
  static double kF = 0.00f;

  public XboxController gp;
  AnalogInput ultrasonic;
  AnalogInput IR;
  AHRS navx;
  DigitalInput limitSwitch;

  @Override
  public void robotInit() {
    
    driveFrontright = new WPI_TalonFX(Statics.TALON_FX_DRIVE_FRONTRIGHT_ID);
    driveBackright = new WPI_TalonFX(Statics.TALON_FX_DRIVE_BACKRIGHT_ID);
    driveBackleft = new WPI_TalonFX(Statics.TALON_FX_DRIVE_BACKLEFT_ID);
    driveFrontleft = new WPI_TalonFX(Statics.TALON_FX_DRIVE_FRONTLEFT_ID);
    shooter = new WPI_TalonFX(Statics.TALON_FX_SHOOTER_ID);
    climb = new WPI_TalonFX(Statics.TALON_FX_CLIMB_ID);
    intakeTop = new VictorSPX(Statics.TALON_FX_INTAKE_TOP_ID);
    intakeBottom = new VictorSPX(Statics.TALON_FX_INTAKE_BOTTOM_ID);
    intakeToshooter = new VictorSPX(Statics.TALON_FX_INTAKE_TOSHOOTER_ID);
    intakeFront = new VictorSPX(Statics.TALON_FX_INTAKE_FRONT_ID);

    shooter.configFactoryDefault();
    driveFrontleft.configFactoryDefault();
    driveBackleft.configFactoryDefault();
    driveFrontright.configFactoryDefault();
    driveBackright.configFactoryDefault();
    driveBackright.follow(driveFrontright);
    driveBackleft.follow(driveFrontleft);
    intakeBottom.follow(intakeTop);

    pneumaticIntake = new DoubleSolenoid(Statics.PNEUMATIC_INTAKE_FORWARD_CHANNEL, Statics.PNEUMATIC_INTAKE_BACKWARD_CHANNEL);
    pneumaticRatchet = new DoubleSolenoid(Statics.PNEUMATIVE_CLIMB_RATCHET_FORWARD_CHANNEL, Statics.PNEUMATIVE_CLIMB_RATCHET_BACKWARD_CHANNEL);
    pneumaticDrivetrainGearshift = new DoubleSolenoid(Statics.PNEUMATIC_DRIVETRAIN_GEARSHIFT_FORWARD_CHANNEL, Statics.PNEUMATIC_DRIVETRAIN_GEARSHIFT_BACKWARD_CHANNEL);

    pneumaticIntake.set(DoubleSolenoid.Value.kReverse);
    pneumaticRatchet.set(DoubleSolenoid.Value.kReverse);
    pneumaticDrivetrainGearshift.set(DoubleSolenoid.Value.kReverse);

    gp = new XboxController(0);
    limitSwitch = new DigitalInput(1);
    IR = new AnalogInput(Statics.IR_FRONT_ID);
    ultrasonic = new AnalogInput(Statics.ULTASONIC_ID);
    navx = new AHRS(); 
    gyroPIDController = new GyroPIDController(Statics.GYRO_P, Statics.GYRO_I, Statics.GYRO_D, Statics.GYRO_F, navx, new gyroPIDOutput());
    
    initializePIDControllers();

    timer = new Timer();

    pdp = new PowerDistributionPanel(0);
    compressor = new Compressor(0);

  }

  @Override
  public void robotPeriodic() {
    
    move(gp.getY(Hand.kLeft), gp.getX(Hand.kRight));
    intake(Statics.intakeSpeed * toInt(gp.getAButton()));
    shooter.set(ControlMode.PercentOutput, Statics.shooterSpeed * toInt(gp.getBButton()));
    if(gp.getXButtonPressed()) togglePneumaticIntake(pneumaticIntake.get());
    if(gp.getBumperPressed(Hand.kLeft)) checkSpeedToggle();
    if(gp.getYButtonPressed()) pneumaticIntake.set(DoubleSolenoid.Value.kReverse);
    
    intakeTop.set(ControlMode.PercentOutput, setBeltSpeed());
    intakeToshooter.set(ControlMode.PercentOutput, setToShooterBeltSpeed());

    runDiagnostics();
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
  
  public void competition1Periodic(){
    updateMove();
    switch(autonStage)
    {
      case 0:
      gyroPIDController.setSetpoint(90);
      break;
    }
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
    drive();
    /*move(gp.getY(Hand.kLeft), gp.getY(Hand.kRight));
    if(gp.getXButtonPressed()) togglePneumaticIntake();
    intake(Statics.intakeSpeed * toInt(gp.getAButton()));
    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));*/
  }

  public void competition4Periodic(){
   drive();
    /* move(gp.getY(Hand.kLeft), gp.getY(Hand.kRight));
    intake(Statics.intakeSpeed * toInt(gp.getAButton()));
    shoot(Statics.shooterSpeed * toInt(gp.getBButton()));*/
  }

  public void test(){
    drive();

    //gyroPIDController.setSetpoint(90);
    //updateMove();
     
  }

  public void checkSpeedToggle()
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
  
  public void togglePneumaticIntake(DoubleSolenoid.Value currentSolenoidValue)
  {
    switch (currentSolenoidValue) {
      case kForward: pneumaticIntake.set(DoubleSolenoid.Value.kReverse);
      break;
      case kReverse: pneumaticIntake.set(DoubleSolenoid.Value.kForward);
      break;
      case kOff: pneumaticIntake.set(DoubleSolenoid.Value.kForward);
      break;
    }
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

  public void move(double leftThrottle, double rightThrottle) {
    //instead of checking for both the positive and the negative versions, just take the absolute value so you only have to check once
    /*if(Math.abs(rightThrottle) >= Statics.stickDeadzone){
      front_right.set(ControlMode.PercentOutput, (rightThrottle * Math.abs(rightThrottle) * speedModifier));
    }
    else {
      front_right.set(ControlMode.PercentOutput, 0);
    }

    if(Math.abs(leftThrottle) >= Statics.stickDeadzone){
      front_left.set(ControlMode.PercentOutput, (-leftThrottle * Math.abs(leftThrottle) * speedModifier));
    }
    else {
      front_left.set(ControlMode.PercentOutput, 0);
    }*/
    double power = -leftThrottle * 0.5;
    double turn = rightThrottle * 0.3;

    double left = power + turn;
    double right = power - turn;

    if(Math.abs(leftThrottle) >= Statics.stickDeadzone){
      driveFrontright.set(ControlMode.PercentOutput, -right);
      driveFrontleft.set(ControlMode.PercentOutput, left);
    }
    else{
      driveFrontright.set(ControlMode.PercentOutput, 0);
      driveFrontleft.set(ControlMode.PercentOutput, 0);
    }
  }

  /*public void indexClear(double speed){
    ballsPresent = 0;
    shooter.set(ControlMode.PercentOutput, speed);
    
    //intakeBottom.set(ControlMode.PercentOutput, -speed);
    intakeToShooter.set(ControlMode.PercentOutput, -speed);
  }*/

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
 
  public void runDiagnostics() {

    SmartDashboard.putNumber("Drive Motor Left", driveFrontright.get());
    SmartDashboard.putNumber("Drive Motor Right", driveFrontleft.get());

    SmartDashboard.putNumber("Intake Front", intakeFront.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Voltage", pdp.getVoltage());

    SmartDashboard.putNumber("Ultrasonic Distance", getRangeInches(ultrasonic.getValue()));
    
    SmartDashboard.putBoolean("Compressor Running", compressor.getPressureSwitchValue());
    SmartDashboard.putBoolean("Compressor is Enabled", compressor.enabled());

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