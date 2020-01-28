package frc.robot.subsystems;

//package frc.team2485.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANError;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.team2485.WarlordsLib.motorcontrol.PIDTalonSRX;
import frc.team2485.WarlordsLib.Limelight;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonSRX;
import frc.team2485.WarlordsLib.sensors.TalonSRXEncoderWrapper;
//import frc.team2485.robot.Constants;
import frc.team2485.WarlordsLib.sensors.WL_Encoder;
import frc.team2485.robot.Constants;


public class Turret extends SubsystemBase{



        private TalonSRX m_talon;

        private TalonSRXEncoderWrapper turretEncoder;

        private Limelight limelight;

        public Turret() {
            m_talon = new TalonSRX(1);

            //turretEncoder = new WL_Encoder(3, 4);

           // turretEncoder.setDistancePerPulse(3 * Math.PI / 4096);

            m_talon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

            limelight = new Limelight(2);

            turretEncoder = new TalonSRXEncoderWrapper(m_talon); //make sure turret encoder is correct
            turretEncoder.setDistancePerRevolution(3 * Math.PI / 4096);

            //fix this


            //SendableRegistry.add(m_talon, "Turret Talon");

            SmartDashboard.putData(this);
        }

        public void setPWM(double pwm) {
            m_talon.set(ControlMode.PercentOutput, pwm);
        }

        public void setPosition(double position) {

        }

        public double getPosition(){
            return turretEncoder.getPosition();
        }

        public Limelight getLimelight(){
            return limelight;
        }

        @Override
        public void periodic() {
            //SmartDashboard.putNumber("Turret Encoder Position", m_encoder.getPosition());
        }
    }

