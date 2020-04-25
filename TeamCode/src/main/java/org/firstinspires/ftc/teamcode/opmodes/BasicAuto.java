package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.util.Safety;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SkystoneRobot;

@Autonomous(name="Basic Autonomous")
public class BasicAuto extends LinearOpMode {

    private SkystoneRobot robot = new SkystoneRobot(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setSafetyMode(Safety.SWIFT);

        telemetry.addData("Robot Position", robot.getRobotPose());

        waitForStart();


    }

}
