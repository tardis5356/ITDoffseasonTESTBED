package org.firstinspires.ftc.teamcode.IMUAutoTest;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;


public class AutoTrajectories {

    //Red


    //Red Specimen Poses


    public static Action spinTest;

    public static final Pose2d StartPos = new Pose2d(0, 0, Math.toRadians(90));



    //Red Basket Actions
//    public static Action redBasket_SubToRightSample;




    public static void generateTrajectories(MecanumDrive drive) {


       spinTest =
                drive.actionBuilder(StartPos)
                        .waitSeconds(3)
                        .turn(Math.toRadians(360))
                        .waitSeconds(3)
                        .turn(Math.toRadians(360))
                        .waitSeconds(3)
                        .turn(Math.toRadians(360))
                        .waitSeconds(3)
                        .build();


    }}