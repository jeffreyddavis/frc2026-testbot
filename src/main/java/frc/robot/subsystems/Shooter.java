package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.addons.LinearServo;

public class Shooter extends SubsystemBase {

    LinearServo m_leftHood = new LinearServo(9, 100);
    public Shooter() {

    }

    public void testServoForward() {
        m_leftHood.extend();
    }

    public void testServoMiddle() {
        m_leftHood.setPositionMm(50);
    }

    public void testServoBackward() {
        m_leftHood.retract();
    }
}
