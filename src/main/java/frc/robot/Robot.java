/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonSRX;
import frc.team2485.WarlordsLib.oi.WL_XboxController;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoderWrapper;
import frc.team2485.WarlordsLib.sensors.WL_Encoder;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private String FILE_PATH = "/home/lvuser/test/csv";

  public Limelight limelight;
  public double tx;
  public boolean tv;
  public static WL_TalonSRX turretTalon;
  public static WL_XboxController stick;
  public static double kP;
  public static double seekingCurrent;
  public static double angleAdjust;
  public WL_Encoder turretEncoder;
  public TalonSRXEncoderWrapper turretEncoderWrapper;
  public int encPort1;
  public int encPort2;
  public double gearRatio;
  public double wheelRadius;
  public double pulsesPerRotation;
  public boolean sweepLeft;
  public boolean sweepRight;
  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    RobotConfigs.getInstance().saveConfigsToFile(FILE_PATH);

    m_robotContainer = new RobotContainer();

    limelight = new Limelight(2);

    turretTalon = new WL_TalonSRX(0, false); //subject to change

    stick = new WL_XboxController(0);

    kP = 0; //change for tuning


    angleAdjust = 0;

    encPort1 = 0; //change
    encPort2 = 1; //change

    gearRatio = 0;
    wheelRadius = 0;
    pulsesPerRotation = 0; //all of these must be changed

    turretEncoder = new WL_Encoder(encPort1, encPort2);

    turretEncoder.setDistancePerPulse(gearRatio * (wheelRadius * 2 * Math.PI) / pulsesPerRotation); //MUST sset up encoder correctly

    turretEncoderWrapper = new TalonSRXEncoderWrapper(turretTalon);

    sweepLeft = false;
    sweepRight = true;
    seekingCurrent = 1; //positive to start

    //IMPORTANT: default homing is to rotate the turret all the way to the left before enabling
    //this way, all the way to the left is 0 degrees and all the way to the right is 315 degrees or something

    


    //turretEncoderWrapper.setDistancePerRevolution(); need to know this
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    RobotConfigs.getInstance().saveConfigsToFile(FILE_PATH);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    turretEncoder.reset();
    turretTalon.clearStickyFaults();

    //make sure turret is placed all the way to the left before enabling... "home it"
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    tv = limelight.hasValidTarget(); //look I used the wrapper
    tx = limelight.getTargetHorizontalOffset(99);

    if(tv){ //vision align
      angleAdjust += tx * kP; //check for += vs =
      turretTalon.set(ControlMode.Current, angleAdjust);
    } else { //seek and sweep across or manual
      if(stick.getXButton()){ //manual control
        turretTalon.set(ControlMode.PercentOutput, stick.getTriggerAxis(GenericHID.Hand.kLeft)); 
      } else if (stick.getAButton()){ //seek
        if(sweepLeft){ //this is like flipping between left and right sweep states
          sweepRight = false;
          seekingCurrent = -1;
        }
    
        if(sweepRight){
          sweepLeft = false;
          seekingCurrent = 1;
        }

       if(sweepRight && turretEncoderWrapper.getPosition() > Math.toRadians(300)){ //GetPos() or getQuadraturePos() ???
         sweepLeft = true;
       }

       if(sweepLeft && turretEncoderWrapper.getPosition() < Math.toRadians(15)){
         sweepRight = true;
       }
        turretTalon.set(ControlMode.Current, seekingCurrent);
       
        }
        
      }

    }

  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
