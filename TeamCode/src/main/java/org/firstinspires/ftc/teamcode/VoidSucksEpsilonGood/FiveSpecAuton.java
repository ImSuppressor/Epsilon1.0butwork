package org.firstinspires.ftc.teamcode.VoidSucksEpsilonGood;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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
        outtakeswivel.setPosition(.5);
        linkl.setPosition(.1);
        linkr.setPosition(.1);
        intakearm.setPosition(.45);
        intakeclaw.setPosition(.48);
        intakeswivel.setPosition(.03);
        clawrotate.setPosition(.5);
        //TODO:init paths
        Action  PlaceSpec1 = drive.actionBuilder(new Pose2d(-64, -7, 0))//place first spec
                .stopAndAdd(new Slidesruntoposition(800))
                .afterTime(1,(new Slidesruntoposition(1500)))
                .strafeToLinearHeading(new Vector2d(-26, -0), 0)
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.4))
                .stopAndAdd(new Setpositionforservo(outtakearml,0.8))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.8))
                .stopAndAdd(new Setpositionforservo(outtakeclaw,0.33))
                .waitSeconds(.1)
                .build();
        Action  MoveSample1 = drive.actionBuilder(new Pose2d(-26, -0, 0))//move first sample
                .stopAndAdd(new Slidesruntoposition(0))
                .stopAndAdd(new Intakeactions(.9, .45, .1, .5, .46))
                .strafeToLinearHeading(new Vector2d(-40, -33), 109.9)
                .turn(-2.48)
                .stopAndAdd(new Intakeactions(.85, .45, .09, .5, .35))
                .build();
        Action  MoveSample2 = drive.actionBuilder(new Pose2d(-40, -33, 0))//move second sample
                .afterTime(.4,(new Intakeactions(.9, .45, .1, .5, .47)))
                .strafeToLinearHeading(new Vector2d(-40, -41.75), 109.95)
                .turn(-2.7)
                .stopAndAdd(new Intakeactions(.9, .45, .1, .5, .35))
                .build();
        Action MoveSample3 = drive.actionBuilder(new Pose2d(-40, -41.75 , 0))//move third sample
                .afterTime(.35,new Intakeactions(.9, .45, .1, .5, .46))
                .strafeToLinearHeading(new Vector2d(-40, -51.5), 109.9)
                .turn(-3)
                .stopAndAdd(new Intakeactions(.45, .45, .03, .5, .1))
                .build();
        Action GetSpec2 = drive.actionBuilder(new Pose2d(-40, -51.5, 0))//move to pickup spec 2
                .stopAndAdd(new Setpositionforservo(outtakearml,0.1))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.1))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.22))
                .strafeToLinearHeading(new Vector2d(-68, -41), 0)
                .stopAndAdd(new Setpositionforservo(outtakeclaw,.6))
                .build();
        Action PlaceSpec2 = drive.actionBuilder(new Pose2d(-68, -41, 0))//Place Specimen 2
                .stopAndAdd(new Setpositionforservo(outtakearml,0.85))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.85))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.5))
                .stopAndAdd(new Slidesruntoposition(800))
                .afterTime(1.65,(new Slidesruntoposition(1500)))
                .strafeToLinearHeading(new Vector2d(-23, -2), 0)
                .stopAndAdd(new Setpositionforservo(outtakearml,0.8))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.8))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.25))
                .waitSeconds(.15)
                .stopAndAdd(new Setpositionforservo(outtakeclaw,0.33))
                .build();
        Action GetSpec3 = drive.actionBuilder(new Pose2d(-24, -2, 0))//move to pickup spec 3
                .stopAndAdd(new Setpositionforservo(outtakearml,0.1))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.1))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.22))
                .stopAndAdd(new Slidesruntoposition(0))
                .strafeToLinearHeading(new Vector2d(-67, -41), 0)
                .stopAndAdd(new Setpositionforservo(outtakeclaw,.6))
                .build();
        Action PlaceSpec3 = drive.actionBuilder(new Pose2d(-67, -41, 0))//Place Specimen 3
                .stopAndAdd(new Setpositionforservo(outtakearml,0.85))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.85))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.5))
                .stopAndAdd(new Slidesruntoposition(800))
                .afterTime(1.65,(new Slidesruntoposition(1500)))
                .strafeToLinearHeading(new Vector2d(-23, -2), 0)
                .stopAndAdd(new Setpositionforservo(outtakearml,0.8))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.8))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.25))
                .waitSeconds(.15)
                .stopAndAdd(new Setpositionforservo(outtakeclaw,0.33))
                .build();
        Action GetSpec4 = drive.actionBuilder(new Pose2d(-23, -2, 0))//move to pickup spec 4
                .stopAndAdd(new Setpositionforservo(outtakearml,0.1))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.1))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.22))
                .stopAndAdd(new Slidesruntoposition(0))
                .strafeToLinearHeading(new Vector2d(-67, -41), 0)
                .stopAndAdd(new Setpositionforservo(outtakeclaw,.6))
                .build();
        Action PlaceSpec4 = drive.actionBuilder(new Pose2d(-67, -41, 0))//Place Specimen 4
                .stopAndAdd(new Setpositionforservo(outtakearml,0.85))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.85))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.5))
                .stopAndAdd(new Slidesruntoposition(800))
                .afterTime(1.65,(new Slidesruntoposition(1500)))
                .strafeToLinearHeading(new Vector2d(-23, -2), 0)
                .stopAndAdd(new Setpositionforservo(outtakearml,0.8))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.8))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.25))
                .waitSeconds(.15)
                .stopAndAdd(new Setpositionforservo(outtakeclaw,0.33))
                .build();
        Action GetSpec5 = drive.actionBuilder(new Pose2d(-23, -2, 0))//move to pickup spec 5
                .stopAndAdd(new Setpositionforservo(outtakearml,0.1))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.1))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.22))
                .stopAndAdd(new Slidesruntoposition(0))
                .strafeToLinearHeading(new Vector2d(-67, -41), 0)
                .stopAndAdd(new Setpositionforservo(outtakeclaw,.6))
                .build();
        Action PlaceSpec5 = drive.actionBuilder(new Pose2d(-67, -41, 0))//Place Specimen 5
                .stopAndAdd(new Setpositionforservo(outtakearml,0.85))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.85))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.5))
                .stopAndAdd(new Slidesruntoposition(800))
                .afterTime(1.65,(new Slidesruntoposition(1500)))
                .strafeToLinearHeading(new Vector2d(-22, -2), 0)
                .stopAndAdd(new Setpositionforservo(outtakearml,0.8))
                .stopAndAdd(new Setpositionforservo(outtakearmr,0.8))
                .stopAndAdd(new Setpositionforservo(outtakeswivel,.25))
                .waitSeconds(.1)
                .stopAndAdd(new Setpositionforservo(outtakeclaw,0.33))
                .stopAndAdd(new Slidesruntoposition(0))
                .waitSeconds(.1)
                .build();
        Action Park = drive.actionBuilder(new Pose2d(-21, -2, 0))//move to park
                .afterTime(0,new Intakeactions(.45, .45, .03, .5, .46))
                .strafeToLinearHeading(new Vector2d(-67, -50), 1.3,
                new TranslationalVelConstraint(150))
                .build();

        waitForStart();

//TODO: WRITE THE CODE HERE YOU MORON

        Actions.runBlocking(new SequentialAction(//place spec 1
                PlaceSpec1
               // PS1,

        ));
        Actions.runBlocking(new SequentialAction(//move samples
                MoveSample1,
                MoveSample2,
                MoveSample3
                )
        );
        Actions.runBlocking(new SequentialAction(//grab spec 2
                GetSpec2
                )

        );
        Actions.runBlocking(new SequentialAction(//place spec 2
                PlaceSpec2
                )
        );
        Actions.runBlocking(new SequentialAction(//grab spec 3
                GetSpec3
                        )
        );
        Actions.runBlocking(new SequentialAction(//place spec 3
                PlaceSpec3
                )
        );
        Actions.runBlocking(new SequentialAction(// grab spec 4
                GetSpec4
                )
        );
        Actions.runBlocking(new SequentialAction(//place spec 4
                PlaceSpec4
                )
        );
        Actions.runBlocking(new SequentialAction(// grab spec 5
                GetSpec5
                )
        );
        Actions.runBlocking(new SequentialAction(//place spec 5
                PlaceSpec5
                )
        );
        Actions.runBlocking(new SequentialAction(// park
                Park
                )
        );
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
                SlideRight.setTargetPosition((int) 420);
                SlideLeft.setTargetPosition((int) 420);
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
                SlideRight.setTargetPosition((int) 440);
                SlideLeft.setTargetPosition((int) 440);
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
                outtakeswivel.setPosition(.5);
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

