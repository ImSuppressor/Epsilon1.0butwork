package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-64, -7, 0))
                        .strafeToLinearHeading(new Vector2d(-28, -0), 0)
                        .waitSeconds(.5)
                        .strafeToLinearHeading(new Vector2d(-40, -33), 109.85)
                        .turn(-2.48)
                        .strafeToLinearHeading(new Vector2d(-40, -41), 109.9)
                        .turn(-2.55)
                        .strafeToLinearHeading(new Vector2d(-40, -48), 109.9)
                        .turn(-3)
                        .strafeToLinearHeading(new Vector2d(-64, -25), 0)
                        .waitSeconds(.5)
                        .strafeToLinearHeading(new Vector2d(-28, -2), 0)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-64, -25), 0)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-28, -2), 0)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-64, -25), 0)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-28, -2), 0)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-64, -25), 0)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-28, -2), 0)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-64, -25), 0)
                .build());

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\Dylan Gibbs\\Downloads\\Juice-INTO-THE-DEEP-Dark")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
