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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.nio.channels.ClosedByInterruptException;

@Autonomous(name="4SampleNoRizzleNorTizzle")
public class FourSampleAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx SlideL = hardwareMap.get(DcMotorEx.class, "slideLeft");
        DcMotorEx SlideR = hardwareMap.get(DcMotorEx.class, "slideRight");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo outtakearml = hardwareMap.servo.get("outtakearmL");
        Servo outtakearmr = hardwareMap.servo.get("outtakearmR");
        Servo intakearm = hardwareMap.servo.get("IntakeArm");
        Servo intakeclaw = hardwareMap.servo.get("IntakeClaw");
        Servo intakeswivel = hardwareMap.servo.get("IntakeSwivel");
        Servo clawrotate = hardwareMap.servo.get("ClawRotate");
        Servo outtakeclaw = hardwareMap.servo.get("outtakeClaw");
        Servo outtakeswivel = hardwareMap.servo.get("outtakeswivel");
        Servo linkr = hardwareMap.servo.get("linkR");
        Servo linkl = hardwareMap.servo.get("LinkL");

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(0, 0), 0)
                        .stopAndAdd(new Setpositionforlink(linkl, linkr, 0))
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
        Servo servo0;
        Servo servo1;
        double position;

        public Setpositionforlink(Servo s0, Servo s1, double p) {
            this.servo0 = s0;
            this.servo1 = s1;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo0.setPosition(position);
            servo1.setPosition(position);

            return false;
        }
    }
    public class Motorruntoposition implements Action {
        DcMotor Lslide;
        DcMotor Rslide;
        double motorsetposition;
        public Motorruntoposition(DcMotor Lslider, DcMotor Rslider, double sp) {
            this.Lslide = Lslider;
            this.Rslide = Rslider;
            this.motorsetposition = sp;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Lslide.setTargetPosition((int) motorsetposition);
            Rslide.setTargetPosition((int) motorsetposition);
            return false;
        }
    }

}
