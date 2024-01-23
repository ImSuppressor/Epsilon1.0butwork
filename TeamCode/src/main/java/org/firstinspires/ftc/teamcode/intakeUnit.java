/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are case sensitive.
 * Motors type: motors for arm and wrist, finger.
 *
 * 1. Arm servo motor: ArmServo
 * 2. wrist servo motor: wristServo
 */
public class intakeUnit
{
    //private
    HardwareMap hardwareMap =  null;

    // wrist servo motor variables
    private Servo fingerServo = null;
    final double FINGER_INTAKE_POS = 0;
    final double FINGER_STOP_POS = 0.5;
    final double FINGER_OUTTAKE_POS = 1.0;

    private Servo switchRightServo = null;
    private Servo switchLeftServo = null;
    final double SWITCH_RIGHT_CLOSE_POS = 0.24;
    final double SWITCH_LEFT_CLOSE_POS = 0.21;
    final double SWITCH_RIGHT_RELEASE = 0.32;
    final double SWITCH_LEFT_RELEASE = 0.05;

    public Servo wristServo = null;
    final double WRIST_MIN_POS = 0.2;  // Minimum rotational position
    final double WRIST_MAX_POS = 0.95; // Maximum rotational position
    final double WRIST_POS_DROP_PURPLE = 0.33;
    final double WRIST_POS_DROP_YELLOW = 0.40;
    final double WRIST_POS_DROP = 0.45;
    final double WRIST_POS_INTAKE = 0.46;

    // arm servo variables, not used in current prototype version.
    public DcMotor armMotor = null;
    int ARM_POS_INTAKE = 3350;//3600; 3560 for finger down, 3600 for finger up
    int ARM_MIN_COUNT_POS = ARM_POS_INTAKE - 3350; //0;
    int ARM_MAX_COUNT_POS = ARM_POS_INTAKE + 100; //3620;
    int ARM_POS_AUTO = ARM_POS_INTAKE - 3300; //80;
    int ARM_POS_HANG = ARM_POS_INTAKE - 3060; //500;
    int ARM_POS_READY_FOR_HANG = ARM_POS_INTAKE - 1760; // 1800
    int ARM_POS_DROP = ARM_POS_INTAKE - 1000; //2550;
    int ARM_POS_CAMERA_READ = ARM_POS_INTAKE - 1060; //2500;
    int ARM_POS_DROP_YELLOW = ARM_POS_INTAKE - 760; //2800;
    int ARM_POS_UNDER_BEAM = ARM_POS_INTAKE - 360; //3200;
    int ARM_POS_DROP_PURPLE = ARM_POS_INTAKE - 180; //3380;
    int ARM_POS_PUSH_PROP = ARM_POS_INTAKE - 100;
    int ARM_POS_INTAKE2 = ARM_POS_INTAKE - 25;
    int ARM_POS_INTAKE3 = ARM_POS_INTAKE - 50;
    int ARM_POS_INTAKE4 = ARM_POS_INTAKE - 75;
    int ARM_POS_INTAKE5 = ARM_POS_INTAKE - 100;


    /**
     * Init slider motors hardware, and set their behaviors.
     * @param hardwareMap the Hardware Mappings.
     * @param armMotorName the name string for arm servo motor
     * @param wristMotorName the name string for wrist servo motor
     * @param switchMotorName the name string for switch servo motor
     */
    public intakeUnit(HardwareMap hardwareMap, String armMotorName, String wristMotorName, String fingerMotorName, String switchMotorName, String switchMotorTwo) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        Logging.log("init motors for finger, wrist and arm.");
        switchRightServo = hardwareMap.get(Servo.class, switchMotorName);
        switchRightServo.setPosition(SWITCH_RIGHT_CLOSE_POS);

        switchLeftServo = hardwareMap.get(Servo.class, switchMotorTwo);
        switchLeftServo.setPosition(SWITCH_LEFT_CLOSE_POS);

        fingerServo = hardwareMap.get(Servo.class, fingerMotorName);
        fingerStop();

        wristServo = hardwareMap.get(Servo.class, wristMotorName);
        wristServo.setDirection(Servo.Direction.FORWARD);
        sleep(200);

        armMotor = hardwareMap.get(DcMotor.class, armMotorName);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSwitchRightPosition(double switchPos) {
        switchPos = Range.clip(switchPos, SWITCH_RIGHT_CLOSE_POS, SWITCH_RIGHT_RELEASE);
        switchRightServo.setPosition(switchPos);
    }

    public void setSwitchLeftPosition(double switchPos){
        switchPos = Range.clip(switchPos, SWITCH_LEFT_RELEASE, SWITCH_LEFT_CLOSE_POS);
        switchLeftServo.setPosition(switchPos);
    }

    public void switchServoOpen() {
        //setSwitchPosition(switchServo.getPosition() + 0.0005);
        setSwitchRightPosition(SWITCH_RIGHT_RELEASE);
        setSwitchLeftPosition(SWITCH_LEFT_RELEASE);
    }

    public void switchServoDropRight() {
        setSwitchRightPosition(SWITCH_RIGHT_RELEASE);
    }

    public void switchServoDropLeft(){
        setSwitchLeftPosition(SWITCH_LEFT_RELEASE);
    }

    public void switchServoClose() {
        setSwitchRightPosition(SWITCH_RIGHT_CLOSE_POS);
        setSwitchLeftPosition(SWITCH_LEFT_CLOSE_POS);
    }

    /**
     * set the target position of wrist servo motor
     * @param wristPos the target position value for wrist servo motor
     */
    public void setWristPosition(double wristPos) {
        wristPos = Range.clip(wristPos, WRIST_MIN_POS, WRIST_MAX_POS);
        wristServo.setPosition(wristPos);
    }
    /**
     * set the wrist servo motor position to open the wrist
     */
    public void wristUp() {
        setWristPosition(wristServo.getPosition() + 0.001);
    }

    /**
     * set the wrist servo motor position to close the wrist
     */
    public void wristDown() {
        setWristPosition(wristServo.getPosition() - 0.001);
    }

    // Finger servo control methods.
    public void fingerIntake() {
        fingerServo.setPosition(FINGER_INTAKE_POS);
    }
    public void fingerStop() {
        fingerServo.setPosition(FINGER_STOP_POS);
    }
    public void fingerOuttake() {
        fingerServo.setPosition(FINGER_OUTTAKE_POS);
    }

    /**
     * set the target position of arm servo motor
     * @param armPos the target position value for arm servo motor
     */
    public void setArmCountPosition(int armPos) {
        //armPos = Range.clip(armPos, ARM_MIN_COUNT_POS, ARM_MAX_COUNT_POS);
        armMotor.setTargetPosition(armPos);
    }

    public void setArmModeRunToPosition(int armPos) {
        setArmCountPosition(armPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.95);
    }
    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmModeRunToPosition(0);

        Logging.log("Arm Motor mode = %s",  armMotor.getMode());
        Logging.log("Arm Motor curr position = %d",  armMotor.getCurrentPosition());
        Logging.log("Arm Motor target position = %d",  armMotor.getTargetPosition());
    }

    public void armUp() {
        setArmCountPosition(armMotor.getCurrentPosition() + 5);
    }

    public void armDown() {
        setArmCountPosition(armMotor.getCurrentPosition() - 5);
    }

    public void armLiftAcc() {
        setArmCountPosition(armMotor.getCurrentPosition() + 50);
    }

    public void armDownAcc() {
        setArmCountPosition(armMotor.getCurrentPosition() - 50);
    }

    public void hangingRobot() {
        setArmCountPosition(ARM_POS_HANG);
    }

    // auto setting positions
    public void intakePositions(int armPosition) {
        setArmCountPosition(armPosition);
        wristServo.setPosition(WRIST_POS_INTAKE);
        switchServoClose();
        fingerIntake();
    }

    public void parkingPositions() {
        setArmCountPosition(ARM_POS_INTAKE);
        wristServo.setPosition(WRIST_POS_INTAKE);
        switchServoClose();
        fingerStop();
    }

    public void dropPositions() {
        setArmCountPosition(ARM_POS_DROP);
        wristServo.setPosition(WRIST_POS_DROP);
        switchServoClose();
        fingerStop();
    }

    public void autonomousInit() {
        setArmCountPosition(ARM_POS_AUTO);
        // wristServo.setPosition(WRIST_POS_AUTO);
        switchServoClose();
        fingerIntake();
    }

    public void readyToDropPurple() {
        setArmCountPosition(ARM_POS_DROP_PURPLE);
        wristServo.setPosition(WRIST_POS_DROP_PURPLE);
        switchServoClose();
        fingerStop();
    }

    public void pushPropPose() {
        setArmCountPosition(ARM_POS_PUSH_PROP);
        wristServo.setPosition(WRIST_POS_INTAKE);
        switchServoClose();
    }

    public void readyToDropYellow(int armPosition){
        setArmCountPosition(armPosition);
        wristServo.setPosition(WRIST_POS_DROP_YELLOW);
        switchServoClose();
        fingerStop();
    }
    public void underTheBeam(){
        setArmCountPosition(ARM_POS_UNDER_BEAM);
        wristServo.setPosition(WRIST_POS_DROP_PURPLE);
    }

    /**
     * Get the arm servo motor current position value
     * @return the current arm servo motor position value
     */
    public int getArmPosition() {
        return armMotor.getCurrentPosition();
    }

    /**
     * Get the wrist servo motor current position value
     * @return the current wrist servo motor position value
     */
    public double getWristPosition() {
        return wristServo.getPosition();
    }

    public double getFingerPosition() {
        return fingerServo.getPosition();
    }

    public double getSwitchRightPosition() {
        return switchRightServo.getPosition();
    }
    public double getSwitchLeftPosition() {
        return switchLeftServo.getPosition();
    }


    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void resetArmPositions(int intakePos) {
        ARM_POS_INTAKE = intakePos;
        ARM_MIN_COUNT_POS = ARM_POS_INTAKE - 3350; //0;
        ARM_MAX_COUNT_POS = ARM_POS_INTAKE + 100; //3620; 
        ARM_POS_AUTO = ARM_POS_INTAKE - 3300; //80;
        ARM_POS_HANG = ARM_POS_INTAKE - 3060; //500;
        ARM_POS_READY_FOR_HANG = ARM_POS_INTAKE - 1760; // 1800
        ARM_POS_DROP = ARM_POS_INTAKE - 1000; //2550;
        ARM_POS_CAMERA_READ = ARM_POS_INTAKE - 1060; //2500;
        ARM_POS_DROP_YELLOW = ARM_POS_INTAKE - 760; //2800;
        ARM_POS_UNDER_BEAM = ARM_POS_INTAKE - 360; //3100;
        ARM_POS_DROP_PURPLE = ARM_POS_INTAKE - 180; //3380;
        ARM_POS_PUSH_PROP = ARM_POS_INTAKE - 100;
        ARM_POS_INTAKE2 = ARM_POS_INTAKE - 25;
        ARM_POS_INTAKE3 = ARM_POS_INTAKE - 50;
        ARM_POS_INTAKE4 = ARM_POS_INTAKE - 75;
        ARM_POS_INTAKE5 = ARM_POS_INTAKE - 100;
    }
}