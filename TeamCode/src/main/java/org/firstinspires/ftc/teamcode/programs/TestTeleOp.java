package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.programs.TestConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    TestConfig robot = new TestConfig();

    public  void runOpMode() throws InterruptedException {

        Components.initialize(this, robot, false, true);
        Components.activateActuatorControl();

        executor.setCommands(
                new Commands.RunResettingLoop(
                        new Commands.RobotCentricMecanumCommand(
                                new Components.BotMotor[] {
                                        leftFront,
                                        leftRear,
                                        rightFront,
                                        rightRear
                                },
                                () -> (double) gamepad1.left_stick_x,
                                () -> (double) gamepad1.left_stick_y,
                                () -> (double) -gamepad1.right_stick_x
                        )
                )
        );
        waitForStart();
        executor.run(this::opModeIsActive);
    }
}