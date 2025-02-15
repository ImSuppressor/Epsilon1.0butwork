package org.firstinspires.ftc.teamcode.VoidSucksEpsilonGood;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="5SpecimenSigmaMode")
public class FiveSpecAuton extends LinearOpMode {
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
        //TODO:init actions for use during trajactories and/or parallel actions
        OuttakeClaw outtakeClaw = new OuttakeClaw(hardwareMap);
        Slides Slides = new Slides(hardwareMap);
        OUTTAKEARM OUTTAKEARM = new OUTTAKEARM(hardwareMap);
        //init
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
        //TODO:init paths
        Action  PlaceSpec1 = drive.actionBuilder(new Pose2d(-64, -7, 0))//place first spec
                .strafeToLinearHeading(new Vector2d(-28, -0), 0)
                .build();
        Action  MoveSample1 = drive.actionBuilder(new Pose2d(-28, -0, 0))//move first sample
                .stopAndAdd(new Intakeactions(.87, .45, .125, .5, .46))
                .strafeToLinearHeading(new Vector2d(-40, -33), 109.85)
                .turn(-2.48)
                .stopAndAdd(new Intakeactions(.87, .45, .125, .5, .35))
                .build();
        Action  MoveSample2 = drive.actionBuilder(new Pose2d(-40, -33, 0))//move second sample
                .afterTime(.75,(new Intakeactions(.87, .45, .125, .5, .46)))
                .strafeToLinearHeading(new Vector2d(-40, -41), 109.9)
                .turn(-2.55)
                .stopAndAdd(new Intakeactions(.87, .45, .125, .5, .35))
                .build();
        Action MoveSample3 = drive.actionBuilder(new Pose2d(-40, -41, 0))//move third sample
                .afterTime(.75,new Intakeactions(.87, .45, .125, .5, .46))
                .strafeToLinearHeading(new Vector2d(-40, -48), 109.9)
                .turn(-3)
                .stopAndAdd(new Intakeactions(.45, .45, .03, .5, .1))
                .build();
        Action GetSpec2 = drive.actionBuilder(new Pose2d(-40, -48, 0))//move to pickup spec 2 TODO:Find out what heading so heading=0 means facing wall
                .strafeToLinearHeading(new Vector2d(-64, -25), -110)
                .build();
        Action GetSpec345 = drive.actionBuilder(new Pose2d(-27.5, -3, 0))//move to pickup spec 3,4,5
                .strafeToLinearHeading(new Vector2d(-50, -30), -110)
                .build();
        Action PlaceSpec2345 = drive.actionBuilder(new Pose2d(-64, -25, 0))//Place Specimen 2,3,4,5
                .strafeToLinearHeading(new Vector2d(-28, -2), -110)
                .build();

        waitForStart();

//TODO: WRITE THE CODE HERE YOU MORON

        Actions.runBlocking(new ParallelAction(
                Slides.slidesSpec(),
                new SequentialAction(
                        PlaceSpec1,
                        Slides.slidesPlaceSpec(),
                        outtakeClaw.outtakeclawopen(),
                        new ParallelAction(
                                Slides.slidesdown(),
                                OUTTAKEARM.grab(),
                                new SequentialAction(
                                        MoveSample1,
                                        MoveSample2,
                                        MoveSample3,
                                        GetSpec2,
                                        outtakeClaw.outtakeclawclose(),
                                        new ParallelAction(
                                                OUTTAKEARM.place(),
                                                Slides.slidesSpec(),
                                                new SequentialAction(
                                                        PlaceSpec2345,
                                                        Slides.slidesPlaceSpec(),
                                                        outtakeClaw.outtakeclawopen(),
                                                        new ParallelAction(//2nd spec placed
                                                                Slides.slidesdown(),
                                                                OUTTAKEARM.grab(),
                                                                new SequentialAction(
                                                                        GetSpec345,
                                                                        outtakeClaw.outtakeclawclose(),
                                                                        new ParallelAction(
                                                                                OUTTAKEARM.place(),
                                                                                Slides.slidesSpec(),
                                                                                new SequentialAction(
                                                                                        PlaceSpec2345,
                                                                                        Slides.slidesPlaceSpec(),
                                                                                        outtakeClaw.outtakeclawopen(),
                                                                                        new ParallelAction(//3rd spec placed
                                                                                                Slides.slidesdown(),
                                                                                                OUTTAKEARM.grab(),
                                                                                                new SequentialAction(
                                                                                                        GetSpec345,
                                                                                                        outtakeClaw.outtakeclawclose(),
                                                                                                        new ParallelAction(//grab 4th spec
                                                                                                                OUTTAKEARM.place(),
                                                                                                                Slides.slidesSpec(),
                                                                                                                new SequentialAction(
                                                                                                                        PlaceSpec2345,
                                                                                                                        Slides.slidesPlaceSpec(),
                                                                                                                        outtakeClaw.outtakeclawopen(),
                                                                                                                        new ParallelAction(//4th spec placed
                                                                                                                                Slides.slidesdown(),
                                                                                                                                OUTTAKEARM.grab(),
                                                                                                                                new SequentialAction(
                                                                                                                                        GetSpec345,
                                                                                                                                        outtakeClaw.outtakeclawclose(),
                                                                                                                                        new ParallelAction(//grab 5th spec
                                                                                                                                                OUTTAKEARM.place(),
                                                                                                                                                Slides.slidesSpec(),
                                                                                                                                                new SequentialAction(
                                                                                                                                                        PlaceSpec2345,
                                                                                                                                                        Slides.slidesPlaceSpec(),
                                                                                                                                                        outtakeClaw.outtakeclawopen(),
                                                                                                                                                        new ParallelAction(//5th spec placed
                                                                                                                                                                Slides.slidesdown(),
                                                                                                                                                                OUTTAKEARM.grab(),
                                                                                                                                                                new SequentialAction(
                                                                                                                                                                        GetSpec345
                                                                                )
                                                                        )

                                                                )
                                                        )
                                                )
                                )
                        )
                )
        )))))))))))));
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
            linkr.setDirection(Servo.Direction.REVERSE);
            linkr.setPosition(linkpos);
            linkl.setPosition(linkpos);
            return false;
        }
    }
    public class OuttakeClaw {
        public Servo outtakeclaw;
        OuttakeClaw(HardwareMap hardwareMap) {
            outtakeclaw = hardwareMap.get(Servo.class, ("outtakeClaw"));
        }
        public class outtakeclawopen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                outtakeclaw.setPosition(.5);
                return false;
            }
        }
        public class outtakeclawclose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                outtakeclaw.setPosition(.4);
                return false;
            }
        }
        public Action outtakeclawopen(){
            return new outtakeclawopen();
        }
        public Action outtakeclawclose() {
            return new outtakeclawclose();
        }
    }
    public class Slides {
        public DcMotorEx SlideRight;
        public DcMotorEx SlideLeft;
        Slides(HardwareMap hardwareMap) {
            SlideRight = hardwareMap.get(DcMotorEx.class, ("slideRight"));
            SlideLeft = hardwareMap.get(DcMotorEx.class, ("slideLeft"));
            SlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class slidesdown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                SlideRight.setTargetPosition((int) 0);
                SlideLeft.setTargetPosition((int) 0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(-1);
                SlideLeft.setPower(-1);
                return false;
            }
        }
        public class slidesSpec implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                SlideRight.setTargetPosition((int) 410);
                SlideLeft.setTargetPosition((int) 410);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(1);
                SlideLeft.setPower(1);
                return false;
            }
        }
        public class slidesPlaceSpec implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                SlideRight.setTargetPosition((int) 420);
                SlideLeft.setTargetPosition((int) 420);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(1);
                SlideLeft.setPower(1);
                return false;
            }
        }
        public Action slidesdown(){
            return new slidesdown();
        }
        public Action slidesSpec() {
            return new slidesSpec();
        }
        public Action slidesPlaceSpec(){
            return new slidesPlaceSpec();
        }
    }
    public class OUTTAKEARM {
        public Servo outtakearmL;
        public Servo outtakearmR;
        public Servo outtakeswivel;
        OUTTAKEARM(HardwareMap hardwareMap) {
            outtakearmL = hardwareMap.get(Servo.class, ("outtakearmL"));
            outtakearmR = hardwareMap.get(Servo.class, ("outtakearmR"));
            outtakeswivel = hardwareMap.servo.get("outtakeSwivel");
            outtakearmR.setDirection(Servo.Direction.REVERSE);
        }
        public class place implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                outtakearmL.setPosition(.85);
                outtakearmR.setPosition(.85);
                outtakeswivel.setPosition(.7);
                return false;
            }
        }
        public class grab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                outtakearmL.setPosition(.01);
                outtakearmR.setPosition(.01);
                outtakeswivel.setPosition(.5);

                return false;
            }
        }
        public Action place(){
            return new place();
        }
        public Action grab(){
            return new place();
        }
    }
}

