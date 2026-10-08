package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.programs.Beecode.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;

@Utility
@TeleOp
public class TestDrivetrain extends LinearOpMode {

    /*
    CONTROLS:

    A - Toggle left back
    B - Toggle left front
    X - Toggle right back
    Y - Toggle right front
     */

    Beecode robot = new Beecode();

    public void runOpMode() throws InterruptedException {

        Components.initialize(this, robot, false, false);
        Components.activateActuatorControl();

        executor.setCommands(
                new Commands.RunResettingLoop(
                        new Commands.RobotCentricMecanumCommand(
                                new Components.BotMotor[] {
                                        leftFront,
                                        leftBack,
                                        rightFront,
                                        rightBack
                                },
                                () -> (double) gamepad1.left_stick_x,
                                () -> (double) gamepad1.left_stick_y,
                                () -> (double) -gamepad1.right_stick_x
                        ),

                        new Commands.PressCommand(

                                new Commands.IfThen(
                                        () -> gamepad1.a,
                                        new Commands.InstantCommand(() -> {
                                            leftBack.setPower(leftBack.getPower() == 0 ? 1 : 0);

                                            Components.telemetry.addLine("Toggled leftBack");
                                            Components.telemetry.addData("Power", leftBack.getPower());
                                            Components.telemetry.addData("Draw (amps)", leftBack.getCurrentAmps());
                                        })
                                ),

                                new Commands.IfThen(
                                        () -> gamepad1.b,
                                        new Commands.InstantCommand(() -> {
                                            leftFront.setPower(leftFront.getPower() == 0 ? 1 : 0);

                                            Components.telemetry.addLine("Toggled leftFront");
                                            Components.telemetry.addData("Power", leftFront.getPower());
                                            Components.telemetry.addData("Draw (amps)", leftFront.getCurrentAmps());
                                        })
                                ),

                                new Commands.IfThen(
                                        () -> gamepad1.x,
                                        new Commands.InstantCommand(() -> {
                                            rightBack.setPower(rightBack.getPower() == 0 ? 1 : 0);

                                            Components.telemetry.addLine("Toggled rightBack");
                                            Components.telemetry.addData("Power", rightBack.getPower());
                                            Components.telemetry.addData("Draw (amps)", rightBack.getCurrentAmps());
                                        })
                                ),

                                new Commands.IfThen(
                                        () -> gamepad1.y,
                                        new Commands.InstantCommand(() -> {
                                            rightFront.setPower(rightFront.getPower() == 0 ? 1 : 0);

                                            Components.telemetry.addLine("Toggled rightFront");
                                            Components.telemetry.addData("Power", rightFront.getPower());
                                            Components.telemetry.addData("Draw (amps)", rightFront.getCurrentAmps());
                                        })
                                )
                        )
                )
        );

        waitForStart();
        executor.run(this::opModeIsActive);
    }
}