package org.firstinspires.ftc.teamcode.VoidSucksEpsilonGood;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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

import java.util.Set;

@Autonomous(name="4SampleNoRizzleNorTizzle")
public class FourSampleAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx SlideL = hardwareMap.get(DcMotorEx.class, "slideLeft");
        DcMotorEx SlideR = hardwareMap.get(DcMotorEx.class, "slideRight");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-64, -7, 0));
        Servo outtakearml = hardwareMap.servo.get("outtakearmL");
        Servo outtakearmr = hardwareMap.servo.get("outtakearmR");
        Servo intakearm = hardwareMap.servo.get("intakeArm");
        Servo intakeclaw = hardwareMap.servo.get("intakeClaw");
        Servo intakeswivel = hardwareMap.servo.get("intakeSwivel");
        Servo clawrotate = hardwareMap.servo.get("clawRotate");
        Servo outtakeclaw = hardwareMap.servo.get("outtakeClaw");
        Servo outtakeswivel = hardwareMap.servo.get("outtakeSwivel");
        Servo linkr = hardwareMap.servo.get("linkR");
        Servo linkl = hardwareMap.servo.get("linkL");
        outtakearmr.setDirection(Servo.Direction.REVERSE);
        linkr.setDirection(Servo.Direction.REVERSE);
        outtakeclaw.setPosition(.6);
        outtakearmr.setPosition(.85);
        outtakearml.setPosition(.85);
        outtakeswivel.setPosition(.7);
        linkl.setPosition(.1);
        linkr.setPosition(.1);
        intakearm.setPosition(.45);
        intakeclaw.setPosition(.48);
        intakeswivel.setPosition(.03);
        clawrotate.setPosition(.5);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-64, -7, 0),
                        new ParallelAction(
                                new Action drive.actionBuilder(new Pose2d(0, 0, 0))
                                        .stopAndAdd(new Slidesruntoposition(30))
                                        .strafeToLinearHeading(new Vector2d(-27.5, -3), 0)
                                        .waitSeconds(.25)
                                        .stopAndAdd(new Setpositionforservo(outtakeclaw, 0.33))
                                        .strafeToLinearHeading(new Vector2d(-40, -30), -110)
                                        .stopAndAdd(new Intakeactions(.8, .45, .2, .5, .47))
                                        .turn(-2.53)
                                        .strafeToLinearHeading(new Vector2d(-50, -30), -110)
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
        public Action
    }

    public class Setpositionforlink implements Action {
        Servo linkr = hardwareMap.servo.get("linkR");
        Servo linkl = hardwareMap.servo.get("linkL");
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
            SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideR.setPower(1);
            SlideL.setPower(1);
            return false;
        }
    }
    public class Intakeactions implements Action {
        Servo intakearm = hardwareMap.servo.get("intakeArm");
        Servo intakeclaw = hardwareMap.servo.get("intakeClaw");
        Servo intakeswivel = hardwareMap.servo.get("intakeSwivel");
        Servo clawrotate = hardwareMap.servo.get("clawRotate");
        Servo linkr = hardwareMap.servo.get("linkR");
        Servo linkl = hardwareMap.servo.get("linkL");
        double setpositionA;
        double setpositionC;
        double setpositionS;
        double setpositionR;
        double linkpos;
        public Intakeactions(double Arm, double Claw, double Swivel, double Rotate, double Linkage) {
            this.setpositionA = Arm;
            this.setpositionC = Claw;
            this.setpositionS = Swivel;
            this.setpositionR = Rotate;
            this.linkpos = Linkage;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakearm.setPosition(setpositionA);
            intakeclaw.setPosition(setpositionC);
            intakeswivel.setPosition(setpositionS);
            clawrotate.setPosition(setpositionR);
            linkl.setDirection(Servo.Direction.REVERSE);
            linkr.setPosition(linkpos);
            linkl.setPosition(linkpos);
            return false;
        }
    }
    public class EmptyClass implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
}

