package org.firstinspires.ftc.teamcode.IMUAutoTest;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

public class ITDAutoTrajectories { public static final Pose2d redSpec_StartPos = new Pose2d(8, -61, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos = new Pose2d(-3, -29, Math.toRadians(90)); //-3
    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(38, -61.0001, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSpecPos1 = new Pose2d(38, -61.0000001, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSpecPos2 = new Pose2d(38, -61.000001, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSpecPos3 = new Pose2d(38, -62.00000001, Math.toRadians(90));
    public static final Pose2d redSpec_MidPointPos = new Pose2d(36, -40, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos1 = new Pose2d(1, -28.5, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos2 = new Pose2d(3, -27, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos3 = new Pose2d(5, -26.5, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos4 = new Pose2d(7, -26, Math.toRadians(90));
    public static final Pose2d redSpecEx_LeftSpecZonePos = new Pose2d(28, -44, Math.toRadians(50));
    public static final Pose2d redSpecEx_LeftSpecDepoPos = new Pose2d(25, -50, Math.toRadians(320));
    public static final Pose2d redSpecEx_MidSpecZonePos = new Pose2d(32.7, -35, Math.toRadians(50));
    public static final Pose2d redSpecEx_MidSpecDepoPos = new Pose2d(32, -47, Math.toRadians(322));
    public static final Pose2d redSpecEx_RightSpecZonePos = new Pose2d(42, -37, Math.toRadians(45));
    public static final Pose2d redSpecEx_RightSpecDepoPos = new Pose2d(33, -58, Math.toRadians(317));
    public static final Pose2d redSpecEx_ObsPrepPos = new Pose2d(40.1, -58, Math.toRadians(90));

    // positions for strafeTo commands
    public static final Vector2d redSpec_ObsSpecPosition3 = new Vector2d(43, -61);

    //Actions
    public static Action redSpec_StartToSub;
    public static Action redSpec_SubToMidPoint;
    public static Action redSpec_ObsToSub1;
    public static Action redSpec_ObsToSub2;
    public static Action redSpec_ObsToSub3;
    public static Action redSpec_ObsToSub4;

    public static Action redSpec_SubToObs;
    public static Action redSpec_SubToObs2;
    public static Action redSpec_SubToObs3;
    public static Action redSpec_SubToObs4;
    public static Action redSpecEx_SubToLeftSpecZone;
    public static Action redSpecEx_LeftSpecToLeftDepo;
    public static Action redSpecEx_LeftDepoToMidSpec;
    public static Action redSpecEx_MidSpecToMidDepo;
    public static Action redSpecEx_MidDepoToRightSpec;
    public static Action redSpecEx_RightSpecToRightDepo;
    public static Action redSpecEx_RightDepoToObsPrep;
    public static Action redSpecEx_ObsPrepToObsSpec;

    public static Action teleop_CurrentToBasket;

    /*
     This defined constraint could also be called slowConstraint or
     something like that, describing it by speed like last year, but
     I do think it could be named after when it gets used during the
     program to better keep track of them (i.e. sweepingConstraint)

     Side note, because I am sure it will come up: I did some digging
     to find out why this is called MinVelConstraint. Essentially, it
     is because it's taking the lower constraint of the two given. At
     any given point, if the robot would be moving more slowly under
     the angular constraint, it will choose that. If it'd be slower
     under the translational constraint, it'd use that.
     */
    public static MinVelConstraint sweepingConstraint = new MinVelConstraint(
            Arrays.asList(
                    new TranslationalVelConstraint(80),//inches per second
                    new AngularVelConstraint(80) // remember the units you're working in, especially for angular constraints!
            )
    );


    public static MinVelConstraint BaseConstraint = new MinVelConstraint(
            Arrays.asList(
                    new TranslationalVelConstraint(80),//inches per second
                    new AngularVelConstraint(6.689) // remember the units you're working in, especially for angular constraints!
            )
    );

    public static MinVelConstraint EndingConstraint = new MinVelConstraint(
            Arrays.asList(
                    new TranslationalVelConstraint(80),//inches per second
                    new AngularVelConstraint(80) // remember the units you're working in, especially for angular constraints!
            )
    );

    /*
     You are also able to define just an accel constraint or just a
     vel constraint, if you only want to constrain one.
     */
    public static AngularVelConstraint sweepingAngularConstraint = new AngularVelConstraint(10);

    /*
     And don't forget about a asymmetric trapezoidal accel constraints!
     I am pretty sure this is a max deceleration of 10in/s and a max
     acceleration of 20in/s, but maybe double check if that seems to
     be the corresponding physical behavior.
     */
    public static ProfileAccelConstraint sweepingAccelProfile = new ProfileAccelConstraint(-80, 80);//(-30,50)
    public static ProfileAccelConstraint SubApproachAccel = new ProfileAccelConstraint(-30, 50);//(-30,50)
    public static ProfileAccelConstraint WallAccel = new ProfileAccelConstraint(-30, 50);//(-25,50)
    public static ProfileAccelConstraint endingAccelProfile = new ProfileAccelConstraint(-80, 80);//(-30,50)


    /*
    See directly applied constraints (I recommend against using them
    though) on line 200.

     Let me know if you need help with this at all, we used these (at
     least their 0.5 equivalent) a bunch in Centerstage, so I'm pretty
     familiar with them   -Graham :)
     */

    public static void generateTrajectories(MecanumDrive drive) {

        redSpec_StartToSub =
                drive.actionBuilder(redSpec_StartPos)
                        .splineToLinearHeading(redSpec_SubDepoPos,Math.toRadians(90),BaseConstraint, SubApproachAccel)
                        //.strafeToLinearHeading(redSpec_SubDepoPosition, Math.toRadians(90))
                        .build();


        redSpec_SubToMidPoint =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_MidPointPos, Math.toRadians(90))
                        .build();


        redSpec_SubToObs =
                drive.actionBuilder(redSpec_SubDepoPos1)
                        .setTangent(Math.toRadians(300))
                        .splineToLinearHeading(redSpec_ObsSpecPos,Math.toRadians(300),BaseConstraint, WallAccel)//270
                        .build();

        redSpec_SubToObs2 =
                drive.actionBuilder(redSpec_SubDepoPos2)
                        .setTangent(Math.toRadians(300))
                        .splineToLinearHeading(redSpec_ObsSpecPos1,Math.toRadians(300),BaseConstraint, WallAccel)//270
                        .build();

        redSpec_SubToObs3 =
                drive.actionBuilder(redSpec_SubDepoPos3)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_ObsSpecPos2,Math.toRadians(300),BaseConstraint, WallAccel)//270
                        .build();

        redSpec_SubToObs4 =
                drive.actionBuilder(redSpec_SubDepoPos4)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_ObsSpecPos2,Math.toRadians(300),EndingConstraint, endingAccelProfile)//270
//                        .strafeToLinearHeading(redSpec_ObsSpecPosition3, Math.toRadians(90), sweepingConstraint, endingAccelProfile)
                        .build();


        redSpec_ObsToSub1 =
                drive.actionBuilder(redSpec_ObsSpecPos)
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(redSpec_SubDepoPos1,Math.toRadians(90),BaseConstraint, SubApproachAccel)
                        //.strafeToLinearHeading(redSpec_SubDepoPosition1, Math.toRadians(90),BaseConstraint, SubApproachAccel)
                        .build();

        redSpec_ObsToSub2 =
                drive.actionBuilder(redSpec_ObsSpecPos1)
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(redSpec_SubDepoPos2,Math.toRadians(90),BaseConstraint, SubApproachAccel)
//                        .strafeToLinearHeading(redSpec_SubDepoPosition2, Math.toRadians(90),BaseConstraint, SubApproachAccel)
                        .build();

        redSpec_ObsToSub3 =
                drive.actionBuilder(redSpec_ObsSpecPos2)
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(redSpec_SubDepoPos3,Math.toRadians(90),BaseConstraint, SubApproachAccel)
//                        .strafeToLinearHeading(redSpec_SubDepoPosition3, Math.toRadians(90),BaseConstraint, SubApproachAccel)
                        .build();

        redSpec_ObsToSub4 =
                drive.actionBuilder(redSpec_ObsSpecPos3)
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(redSpec_SubDepoPos4,Math.toRadians(90),BaseConstraint, SubApproachAccel)
//                        .strafeToLinearHeading(redSpec_SubDepoPosition4, Math.toRadians(90),BaseConstraint, SubApproachAccel/*, sweepingConstraint, endingAccelProfile*/)
                        .build();

        redSpecEx_SubToLeftSpecZone =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpecEx_LeftSpecZonePos, Math.toRadians(0))
                        .build();


        /*
         This shows how you can add the constraints to just a
         single trajectory, but I'd recommend making them
         predefined and reusable (see below)
         */
        redSpecEx_LeftSpecToLeftDepo =
                drive.actionBuilder(redSpecEx_LeftSpecZonePos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redSpecEx_LeftSpecDepoPos, Math.toRadians(0), sweepingConstraint, sweepingAccelProfile)
                        .build();

        redSpecEx_LeftDepoToMidSpec =
                drive.actionBuilder(redSpecEx_LeftSpecDepoPos)
                        .setTangent(Math.toRadians(90))//0
                        .splineToLinearHeading(redSpecEx_MidSpecZonePos, Math.toRadians(0), sweepingConstraint, sweepingAccelProfile)
                        .build();


        redSpecEx_MidSpecToMidDepo =
                drive.actionBuilder(redSpecEx_MidSpecZonePos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redSpecEx_MidSpecDepoPos, Math.toRadians(270), sweepingConstraint, sweepingAccelProfile)
                        .build();


        redSpecEx_MidDepoToRightSpec =
                drive.actionBuilder(redSpecEx_MidSpecDepoPos)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(redSpecEx_RightSpecZonePos, Math.toRadians(0), sweepingConstraint, sweepingAccelProfile)
                        .build();

        redSpecEx_RightSpecToRightDepo =
                drive.actionBuilder(redSpecEx_RightSpecZonePos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redSpecEx_RightSpecDepoPos, Math.toRadians(270), sweepingConstraint, sweepingAccelProfile)
                        .build();


        redSpecEx_RightDepoToObsPrep =
                drive.actionBuilder(redSpecEx_RightSpecDepoPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpecEx_ObsPrepPos, Math.toRadians(270))
                        .build();

        redSpecEx_ObsPrepToObsSpec =
                drive.actionBuilder(redSpecEx_ObsPrepPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                        .build();


    }
}
