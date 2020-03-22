package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class YellowJacket435 implements Motor {

    private DcMotor m_motor;
    private PIDFController controller;

    public YellowJacket435(HardwareMap hMap, String name, boolean isInverted) {
        m_motor = hMap.get(DcMotor.class, name);
        setInverted(isInverted);
    }

    public YellowJacket435(HardwareMap hMap, String name) {
        this(hMap, name, false);
    }

    @Override
    public void set(double speed) {
        m_motor.setPower(speed);
    }

    @Override
    public double get() {
        return m_motor.getPower();
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_motor.setDirection(isInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    @Override
    public boolean getInverted() {
        return m_motor.getDirection() == DcMotor.Direction.REVERSE;
    }

    @Override
    public void disable() {
        m_motor.close();
    }

    @Override
    public String getDeviceType() {
        return "Yellow Jacket, 435 rpm";
    }

    /**
     * Setting the speed in terms of rotations per minute (rpm)
     *
     * @param speed the speed in rpm
     */
    public void setSpeed(double speed) {
        set(speed / 435);
    }

    /**
     * @param output percentage of output speed
     */
    @Override
    public void pidWrite(double output) {
        set(controller.calculate(output, get()));
    }

    @Override
    public void stopMotor() {
        set(0);
    }

}
