package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Components.*;

import java.util.ArrayList;

public class Beecode implements RobotConfig {
    public static BotMotor leftFront = new BotMotor("leftFront", Direction.REVERSE);
    public static BotMotor leftBack = new BotMotor("leftBack", Direction.REVERSE);
    public static BotMotor rightFront = new BotMotor("rightFront", Direction.FORWARD);
    public static BotMotor rightBack = new BotMotor("rightBack", Direction.FORWARD);
    @Override
    public ArrayList<Actuator<?>> getActuators() {
        return null;
    }
}
