package org.firstinspires.ftc.teamcode.VoidSucksEpsilonGood;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="4SampleNoRizzleNorTizzle")
public class FourSampleAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx SlideL = hardwareMap.get(DcMotorEx.class, "slideLeft");
        DcMotorEx SlideR = hardwareMap.get(DcMotorEx.class, "slideRight");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo outtakearml = hardwareMap.servo.get("outtakearmL");
        Servo outtakearmr = hardwareMap.servo.get("outtakearmR");
        Servo intakearm = hardwareMap.servo.get("intakeArm");
        Servo intakeclaw = hardwareMap.servo.get("intakeClaw");
        Servo intakeswivel = hardwareMap.servo.get("intakeSwivel");
        Servo clawrotate = hardwareMap.servo.get("clawRotate");
        Servo outtakeclaw = hardwareMap.servo.get("outtakeClaw");
        Servo outtakeswivel = hardwareMap.servo.get("outtakeswivel");
        Servo linkr = hardwareMap.servo.get("linkR");
        Servo linkl = hardwareMap.servo.get("LinkL");

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new Slidesruntoposition(15))
                        .strafeToLinearHeading(new Vector2d(15, 15), 0)
                        .waitSeconds(5)
                        .build());
    }
    public class Setpositionforservo implements Action {
        Servo servo;
        double position;
        public Setpositionforservo(Servo s, double p) {
            this.servo = s;
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
        }
    }
    public class Setpositionforlink implements Action {
        Servo linkr = hardwareMap.servo.get("linkR");
        Servo linkl = hardwareMap.servo.get("LinkL");
        double InRobot;
        double position;
        public Setpositionforlink(double p) {
            this.position = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if ((position) == InRobot) {
                linkl.setDirection(Servo.Direction.REVERSE);
                linkr.setPosition(.12);
                linkl.setPosition(.12);
                return false;
            } else {
                //find out if linkl or linkr reversed and correct. Same with slides
                linkl.setDirection(Servo.Direction.REVERSE);
                linkr.setPosition(position);
                linkl.setPosition(position);
                return false;
            }
        }
    }
        public class Slidesruntoposition implements Action {
            DcMotorEx SlideL = hardwareMap.get(DcMotorEx.class, "slideLeft");
            DcMotorEx SlideR = hardwareMap.get(DcMotorEx.class, "slideRight");
            double motorsetposition;
            public Slidesruntoposition(double sp) {
                this.motorsetposition = sp;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //find out which motor reversed
                SlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                SlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                SlideL.setDirection(DcMotorSimple.Direction.REVERSE);
                SlideR.setTargetPosition((int) motorsetposition);
                SlideL.setTargetPosition((int) motorsetposition);
                SlideR.setPower(1);
                SlideL.setPower(1);
                return false;
            }
        }
    }

