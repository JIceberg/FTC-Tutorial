package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.Robot;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.JSTEncoder;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.util.Safety;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motors.YellowJacket435;
import org.firstinspires.ftc.teamcode.hardware.motors.YellowJacketEx;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeTwoWheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SinglePulleyLift;

public class SkystoneRobot extends Robot {

    private YellowJacket435 m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MecanumDrive m_drive;

    private JSTEncoder m_encoderLeft, m_encoderRight, m_centralEncoder;

    private YellowJacket435 m_intakeLeft, m_intakeRight;
    private YellowJacketEx m_liftMotor;

    private IntakeTwoWheel m_intake;
    private SinglePulleyLift m_lift;

    private RevIMU m_imu;
    private HolonomicOdometry m_odometry;

    /**
     * The constructor, which constructs the hardware for the robot
     * @param hardwareMap the hardware map from the opmode
     */
    public SkystoneRobot(HardwareMap hardwareMap) {
        m_frontLeft = new YellowJacket435(hardwareMap, "fl");
        m_frontRight = new YellowJacket435(hardwareMap, "fr");
        m_backLeft = new YellowJacket435(hardwareMap, "bl");
        m_backRight = new YellowJacket435(hardwareMap, "br");

        m_intakeLeft = new YellowJacket435(hardwareMap, "il");
        m_intakeRight = new YellowJacket435(hardwareMap, "ir");

        m_liftMotor = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "lift")
        );

        m_drive = new MecanumDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

        m_lift = new SinglePulleyLift(m_liftMotor);
        m_intake = new IntakeTwoWheel(m_intakeLeft, m_intakeRight);

        m_imu = new RevIMU(hardwareMap);
        m_odometry = new HolonomicOdometry(16.77); // our trackwidth is 16.77 inches

        m_encoderLeft = new JSTEncoder(hardwareMap, "encoderLeft");
        m_encoderRight = new JSTEncoder(hardwareMap, "encoderRight");
        m_centralEncoder = new JSTEncoder(hardwareMap, "encoderCenter");

        m_encoderLeft.setDistancePerPulse(2.0 * Math.PI / 8092); // rev through-bore encoder
        m_encoderRight.setDistancePerPulse(2.0 * Math.PI / 8092);
        m_centralEncoder.setDistancePerPulse(2.0 * Math.PI / 8092);
    }

    public void drive(double x, double y, double turn) {
        if (m_safety == Safety.SWIFT) {
            m_drive.driveFieldCentric(
                    x, y, turn, Math.toRadians(m_imu.getHeading())
            );
        } else {
            m_drive.driveFieldCentric(
                    x, y, turn, Math.toRadians(m_imu.getHeading()), true
            );
        }
    }

    public void intake() {
        m_intake.intake();
    }

    public void outtake() {
        m_intake.outtake();
    }

    public void stopIntake() {
        m_intake.stop();
    }

    public void actuateLift(double speed) {
        m_lift.lift(speed);
    }

    public void disable() {
        m_drive.stopMotor();
        m_lift.disable();
        m_intake.disable();
    }

    public Pose2d getRobotPose() {
        m_odometry.update(
                Math.toRadians(m_imu.getHeading()),
                m_centralEncoder.getDistance(),
                m_encoderLeft.getDistance(),
                m_encoderRight.getDistance()
        );
        return m_odometry.getPose();
    }

}
