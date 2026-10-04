package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple.*;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.Components.*;
import org.firstinspires.ftc.teamcode.presets.PresetControl.*;

import java.util.ArrayList;
import java.util.Arrays;
import static java.lang.Math.*;

public class Beecode implements RobotConfig {
    public static BotMotor leftFront = new BotMotor("leftFront", Direction.REVERSE);
    public static BotMotor leftBack = new BotMotor("leftBack", Direction.REVERSE);
    public static BotMotor rightFront = new BotMotor("rightFront", Direction.FORWARD);
    public static BotMotor rightBack = new BotMotor("rightBack", Direction.FORWARD);
    public static BotMotor intake = new BotMotor("intakeMotor", Direction.FORWARD);
    public static BotMotor transfer = new BotMotor("transferMotor", Direction.FORWARD);
    public static BotMotor turret = new BotMotor("turretMotor", Direction.FORWARD).setControlSystems(new String[]{"PID"}, new ControlSystem<BotMotor>(new PID(0.015, 0,0)));
    public static BotMotor flywheel;
    static {
        flywheel = new BotMotor("flywheelMotor", Direction.FORWARD).setControlSystems(new String[]{"PID"}, new ControlSystem<BotMotor>(()->flywheel.getVelocity(), new PID(0.015, 0,0), new BasicFeedforward(0)));
    }
    public static BotServo transferGate = new BotServo("transferGate", Servo.Direction.FORWARD, 0, 180).setKeyTargets(new String[]{"open","closed"}, new double[]{0,0});
    public static BotServo hood = new BotServo("hood", Servo.Direction.FORWARD, 0, 180);
    public static BotServo ramp = new BotServo("hood", Servo.Direction.FORWARD, 0, 180).setKeyTargets(new String[]{"down","up"}, new double[]{0,0});
    @Override
    public ArrayList<Actuator<?>> getActuators() {return new ArrayList<>(Arrays.asList(leftFront, leftBack, rightFront, rightBack, intake, transfer, turret, flywheel, transferGate, hood, ramp));}

    public static boolean isShooting = false;
    public static boolean isIntaking = false;
    public enum BallCount {
        EMPTY,
        ATMOSTONE,
        ATLEASTTWO,
        FULL
    }
    public static BallCount count = BallCount.EMPTY;
    public static double FIELDWIDTH = 141.5;
    public static class Hive{
        public static boolean shootSide = true;
        public static double hiveAngle = 30;
        public static final double FULCRUM_HEIGHT = 43.95;
        public static final double FULCRUMX = FIELDWIDTH/2-12.75;
        public static final double SUB_HEIGHT = 0-POLLENRADIUS;
        public static final double BAR_LENGTH = 21.46;
        public static double [] referencePoint = new double[3];
        public static double [] offsets = new double[3];
        public static double [] targetPoint = new double[3];
        public static void setReferencePoint(){
            referencePoint[0] = FULCRUMX;
            double sideAngle = shootSide ? hiveAngle : 180+hiveAngle;
            double sideHyp = sqrt(SUB_HEIGHT*SUB_HEIGHT+BAR_LENGTH*BAR_LENGTH);
            double fulcrumToPointAngle = shootSide ? toRadians(sideAngle) - atan(SUB_HEIGHT/BAR_LENGTH) : toRadians(sideAngle) + atan(SUB_HEIGHT/BAR_LENGTH);
            referencePoint[1] = cos(fulcrumToPointAngle)*sideHyp + FIELDWIDTH/2;
            referencePoint[2] = sin(fulcrumToPointAngle)*sideHyp + FULCRUM_HEIGHT;
        }
        public static void calcTargetOffsets(){

        }
        public static void setTargetPoint(){
            targetPoint[0] = shootSide ? referencePoint[0] - offsets[0] : referencePoint[0] + offsets[0];
            double sideAngle = shootSide ? hiveAngle : 180+hiveAngle;
            double offsetHyp = sqrt(offsets[1]*offsets[1] + offsets[2]*offsets[2]);
            double offsetAngle = Math.toDegrees(atan2(offsets[2], shootSide ? -offsets[2] : offsets[2]));
            targetPoint[1] = offsetHyp*cos(Math.toRadians(sideAngle+offsetAngle));
            targetPoint[2] = offsetHyp*sin(Math.toRadians(sideAngle+offsetAngle));
        }
    }

    public static double TURRETHEIGHT = 0; public static double TURRETTOBACK = 0; public static double TURRETTOFRONT = 0; public static double TURRETTOSIDE = 0;
    public static double INTAKESPAN = 0; public static double RAMPLENGTH = 0; public static double RAMPWIDTH = 0;
    public static double POLLENRADIUS = 1.4;
    public void generalInit(){
        Components.enableCaching(LynxModule.BulkCachingMode.MANUAL);
        executor.setClearBulkCache(true);
        isShooting = false; isIntaking = false;
    }
}
