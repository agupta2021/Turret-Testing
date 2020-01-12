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
    // public WL_Encoder turretEncoder;
    public TalonSRXEncoderWrapper turretEncoderWrapper;
    public int encPort1;
    public int encPort2;
    public double gearRatio;
    public double wheelRadius;
    public double pulsesPerRotation;
    public boolean sweepLeft;
    public boolean sweepRight;
    public double steerCommand;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        RobotConfigs.getInstance().saveConfigsToFile(FILE_PATH);

        m_robotContainer = new RobotContainer();

        limelight = new Limelight(2);

        turretTalon = new TalonSRX(1); // subject to change


        stick = new WL_XboxController(0);

        kP = 0.03; // change for tuning

        angleAdjust = 0;

        // encPort1 = 0; //change
        // encPort2 = 1; //change

        gearRatio = 1;
        wheelRadius = 1;
        pulsesPerRotation = 4096; // all of these must be changed

        // turretEncoder = new WL_Encoder(encPort1, encPort2);

        // turretEncoder.setDistancePerPulse(gearRatio * (wheelRadius * 2 * Math.PI) /
        // pulsesPerRotation); //MUST sset up encoder correctly

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

        steerCommand = 0;
        //r.init();



    turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    turretTalon.getSensorCollection().setPulseWidthPosition(0, 50);
        // make sure turret is placed all the way to the left before enabling... "home
        // it"
        sweepLeft = false;
        sweepRight = true;
        seekingCurrent = 0.5;
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
      //r.run();
        tv = limelight.hasValidTarget(); // look I used the wrapper
        tx = limelight.getTargetHorizontalOffset(0);
        // System.out.println("tv:" + tv);
        //  System.out.println("tx:" + tx);
        //System.out.println("angle adjust:" + angleAdjust);


        double position = Math.PI * (turretTalon.getSelectedSensorPosition()/2048.0);

        SmartDashboard.putNumber("Sensor Pos Rad", position);

        SmartDashboard.putNumber("Sensor Pos Deg", Math.toDegrees(position));

        // System.out.println("WrapperPos" + turretEncoderWrapper.getPosition());
        if (tv && stick.getYButton()) { //vision align ... take Y button out
            // System.out.println("yeeeeeet");
            angleAdjust = tx * kP; //check for += vs =
            turretTalon.set(ControlMode.PercentOutput, angleAdjust);
        } else { //seek and sweep across or manual

            if (stick.getXButton()) { // manual control

                steerCommand = stick.getX(GenericHID.Hand.kLeft) * stick.getX(GenericHID.Hand.kLeft);

                if (stick.getX(GenericHID.Hand.kLeft) < 0) {
                    steerCommand *= -1;
                }

                if (steerCommand > 0.5) {
                    steerCommand = 0.5;
                }
                if (steerCommand < -0.5) {
                    steerCommand = -0.5;
                }

                turretTalon.set(ControlMode.PercentOutput, steerCommand);


                //System.out.println("current:" + turretTalon.getStatorCurrent());

            }
            else if (stick.getAButton()){ //seek


              if(sweepRight && position > Math.toRadians(300)){
                //GetPos() or getQuadraturePos() ???
                sweepLeft = true;
                sweepRight = false;
                seekingCurrent = -0.5;
              }

              if(sweepLeft && position < Math.toRadians(15)){
                sweepRight = true;
                sweepLeft = false;
                seekingCurrent = 0.5;
              }


              turretTalon.set(ControlMode.PercentOutput, seekingCurrent);

            }
            else {
                turretTalon.set(ControlMode.PercentOutput, 0);
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
