package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.programs.BeecodeConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Commands.*;
import org.firstinspires.ftc.teamcode.base.Components;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class BeecodeTeleOp extends LinearOpMode {
    BeecodeConfig robot = new BeecodeConfig();

    public void telemetryFunction() {
        Components.telemetry.addLine("RightFront");
        Components.telemetry.addData("Target Set Power: ", rightFront.getPower());
        Components.telemetry.addData("Actual Velocity: ", rightFront.getVelocity());
        Components.telemetry.addData("Draw (amps)", rightFront.getCurrentAmps());

        Components.telemetry.addLine("\nRightBack");
        Components.telemetry.addData("Target Set Power: ", rightBack.getPower());
        Components.telemetry.addData("Actual Velocity: ", rightBack.getVelocity());
        Components.telemetry.addData("Draw (amps)", rightBack.getCurrentAmps());

        Components.telemetry.addLine("\nToggled leftFront");
        Components.telemetry.addData("Target Set Power: ", leftFront.getPower());
        Components.telemetry.addData("Actual Velocity: ", leftFront.getVelocity());
        Components.telemetry.addData("Draw (amps)", leftFront.getCurrentAmps());

        Components.telemetry.addLine("\nLeftBack");
        Components.telemetry.addData("Target Set Power: ", leftBack.getPower());
        Components.telemetry.addData("Actual Velocity: ", leftBack.getVelocity());
        Components.telemetry.addData("Draw (amps)", leftBack.getCurrentAmps());

        Components.telemetry.addLine("\nTransfer");
        Components.telemetry.addData("Set Power: ", transfer.getVelocity());
        Components.telemetry.addData("Actual Velocity: ", transfer.getVelocity());

        Components.telemetry.addLine("\nFlywheel");
        Components.telemetry.addData("Set Power: ", flywheel.getVelocity());
        Components.telemetry.addData("Actual Velocity: ", flywheel.getVelocity());

        Components.telemetry.addLine("\nIntake");
        Components.telemetry.addData("Set Power: ", intake.getVelocity());
        Components.telemetry.addData("Actual Velocity: ", intake.getVelocity());
        Components.telemetry.addData("Draw (amps)", intake.getCurrentAmps());



        telemetry.update();
    }


    public  void runOpMode() throws InterruptedException {

        Components.initialize(this, robot, false, true);
        Components.activateActuatorControl();

        executor.setCommands(
            new Commands.RunResettingLoop(
                updateIntakeTransfer,
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
                new PressCommand(
                        new IfThen(
                                ()->gamepad1.left_trigger_pressed,
                                new InstantCommand(
                                        ()->isIntaking=!isIntaking
                                )
                        ),
                        new IfThen(
                                ()->gamepad1.right_trigger_pressed,
                                new InstantCommand(()->{
                                    if (count==BallCount.EMPTY) count = BallCount.PARTIAL;
                                    isShooting=true;
                                })
                        )
                ), new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.a,
                            new Commands.InstantCommand(() -> {
                                leftBack.setPower(leftBack.getPower() == 0 ? 1 : 0);
                            })
                    )
                ), new Commands.PressCommand(
                        new Commands.IfThen(() -> gamepad1.b,
                                new Commands.InstantCommand(() -> {
                                    leftBack.setPower(leftBack.getPower() == 0 ? 1 : 0);
                                })
                        )
                ), new Commands.PressCommand(
                        new Commands.IfThen(() -> gamepad1.x,
                                new Commands.InstantCommand(() -> {
                                    leftBack.setPower(leftBack.getPower() == 0 ? 1 : 0);
                                })
                        )
                ), new Commands.PressCommand(
                        new Commands.IfThen(() -> gamepad1.y,
                                new Commands.InstantCommand(() -> {
                                    leftBack.setPower(leftBack.getPower() == 0 ? 1 : 0);
                                })
                        )
                )
            )
        );
                 /*, new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.right_bumper,
                        new Commands.InstantCommand(() -> {
                            hood.setTarget(180);;
                        })
                    )
                ), new Commands.PressCommand(
                    new Commands.IfThen(() -> !gamepad1.right_bumper,
                            new Commands.InstantCommand(() -> {
                                hood.setTarget(0);;
                            })
                    )
                ), new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.right_stick_button,
                        new Commands.InstantCommand(() -> {
                            ramp.setTarget(180);;
                        })
                    )
                ), new Commands.PressCommand(
                    new Commands.IfThen(() -> !gamepad1.right_stick_button,
                            new Commands.InstantCommand(() -> {
                                ramp.setTarget(0);;
                            })
                    )
                )*/
        waitForStart();
        executor.run(this::opModeIsActive);
    }
}