/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.PeriodicRunner;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonSRX;
import frc.team2485.WarlordsLib.oi.Deadband;
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
    public static TalonSRX turretTalon;
    public static WL_XboxController stick;
    public static double kP;
    public static double seekingCurrent;
    public static double angleAdjust;
    public WL_Encoder turretEncoder;
    public TalonSRXEncoderWrapper turretEncoderWrapper;
    public int encPort1 = 3;
    public int encPort2 = 4;
    public double gearRatio;
    public double wheelRadius;
    public double pulsesPerRotation;
    public boolean sweepLeft;
    public boolean sweepRight;
    public double steerCommand;
    public TalonSRX m_talonThing;
    public TalonSRXEncoderWrapper turretEncoderThing;



    public static double lastAlignedHeading;
    public static double lastTyVal;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        RobotConfigs.getInstance().saveConfigsToFile(FILE_PATH);
        //ignore most of this stuff
        m_robotContainer = new RobotContainer();

        limelight = new Limelight(2);

        turretTalon = new TalonSRX(1); // subject to change

        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

        turretEncoderThing = new TalonSRXEncoderWrapper(turretTalon);



        turretEncoderThing.setDistancePerRevolution(3 * Math.PI / 4096);



        stick = new WL_XboxController(0);

        kP = 0.03; // change for tuning

        angleAdjust = 0;
        lastAlignedHeading = 0;
        lastTyVal = 0;

        // encPort1 = 0; //change
        // encPort2 = 1; //change

        gearRatio = 1;
        wheelRadius = 1;
        pulsesPerRotation = 4096; // all of these must be changed

        turretEncoder = new WL_Encoder(encPort1, encPort2);

        turretEncoder.setDistancePerPulse(3 * Math.PI / 4096);

        // turretEncoderWrapper = new TalonSRXEncoderWrapper(turretTalon);

         // positive to start
        turretTalon.clearStickyFaults();

        // IMPORTANT: default homing is to rotate the turret all the way to the left
        // before enabling
        // this way, all the way to the left is 0 degrees and all the way to the right
        // is 315 degrees or something

        // turretEncoderWrapper.setDistancePerRevolution(); need to know this

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        System.out.println("Turret Encoder Val" + turretEncoderThing.getPosition());
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
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
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

//    PeriodicRunner r = new PeriodicRunner(() -> {
//        System.out.println("SelectedSensorPos " +
//                (turretTalon.getSensorCollection().getPulseWidthPosition() / 1024.0));
//    }, 0.5);

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // turretEncoder.reset();

       // steerCommand = 0;

        //r.init();



    //turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    //turretTalon.getSensorCollection().setPulseWidthPosition(0, 50);
        // make sure turret is placed all the way to the left before enabling... "home
        // it"
//        sweepLeft = false;
//        sweepRight = true;
//        seekingCurrent = 0.5;
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
      //r.run();







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
