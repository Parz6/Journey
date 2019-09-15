package Journey.util;

import Journey.input.CarData;
import Journey.input.DataPacket;
import Journey.output.ControlsOutput;
import Journey.vector.Vector2;
import Journey.vector.Vector3;
import rlbot.cppinterop.RLBotDll;
import rlbot.flat.BoostOption;

public class Misc {
    private Misc() {} // Disable instantiation

    public static final double frameT = 1.0 / 60.0; // Expected frame time (60Hz period)
    public static final double ballR = 92.75;       // Ball radius
    public static final double ballContactR = 86;   // For rolling ball and grounded car

    public static final double octaneHalfLength = 59.003689;
    public static final double octaneHalfWidth = 42.099705;
    public static final double octaneLengthoffset = 13.87566;
    public static final double octaneFrontTopHeight = 55.14;
    public static final double octaneHalfHeight = 18.079536;
    public static final double octaneJointToTop = 38.83;
    public static final double octaneJointHeight = 17.01; // Car z position at rest on ground

    public static final double netHeight = 642.775 / 2.0;
    public static final double netHalfWidth = 892.755; // From center to post
    public static final double netY = 5120;
    public static final double wallX = 4096;
    public static final Vector3 blueNetPos = new Vector3(0, -netY, netHeight / 2.0);    // team=0
    public static final Vector3 orangeNetPos = new Vector3(0, netY, netHeight / 2.0);   // team=1

    public static final int minWaitFramesBeforeFlip = 3; // Min frames of no jump between initial jump and second jump for flip
    public static final double postFlipWheelContactTimeout = 0.125; // To disable air recovery etc. while flip is in progress
    public static final double postFlipMaxWaitTime = 1.3; //0.6? "


    // Sign functions
    public static int sgn(int val) {
        return Integer.signum(val);
    }
    public static double sgn(double val) {
        return Math.signum(val);
    }
    public static float sgn(float val) {
        return Math.signum(val);
    }
    public static double sgnNo0(double val) { // Default to 1 if val is zero
        return val != 0 ? sgn(val) : 1;
    }

    // Returns val constrained between min and max (or +-bound)
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    public static double clamp(double val, double bound) {
        return bound > 0 ? clamp(val, -bound, bound) : clamp(val, bound, -bound);
    }

    // Acceleration due to full throttle, scales down linearly with throttle (0,1] ([-1,0) if car moving backwards)
    public static double throttleAccelAtSpd(double s) { //Full (1.0) throttle accel at speed s
        if (s < 0)
            return 3500;
        if (s == 0)
            return 1600;
        if (s < 1400) // 0 < |s| < 1400
            return 1592.0 - (s * (1432.0 / 1400.0)); // 1600 - (1440 * (|s| / 1400)); 1440 / 1400 = 36.0 / 35.0
        if (s < 1410) // 1400 <= |s| < 1410
            return 22560.0 - (s * 16.0); // 160 * ((1410 - |s|) / 10)
        return 0; // |s| >= 1410
    }
    // Curvature should be directly proportional to steer: kActual = k*abs(steer)
    public static double turnRadiusForSpeed(double s) {
        final double[] speeds = {0.0, 500.0, 1000.0, 1500.0, 1750.0, 2300.0};
        final double[] curvatures = {0.0069, 0.00398, 0.00235, 0.001375, 0.0011, 0.00088};
        s = Misc.clamp(s, speeds[0], speeds[5]);
        int i;
        for (i = 0; i < 5; i++) {
            if (s <= speeds[i + 1])
                break;
        }
        //System.out.println("i: " + i + ", s:" + s);
        double speedRange = speeds[i] - speeds[i + 1];
        double curvatureRange = curvatures[i + 1] - curvatures[i];
        double proportion = (speeds[i] - s) / speedRange;
        double k = curvatures[i] + (proportion * curvatureRange);
        return 1.0 / Misc.clamp(k, curvatures[curvatures.length - 1], curvatures[0]);
    }
    // This works and is based off Chip's data
    public static double speedForTurnRadius(double r) {
        final double[] speeds = {0.0, 500.0, 1000.0, 1500.0, 1750.0, 2300.0};
        final double[] curvatures = {0.0069, 0.00398, 0.00235, 0.001375, 0.0011, 0.00088};
        double k = Misc.clamp(1.0 / Math.abs(r), curvatures[curvatures.length - 1], curvatures[0]); //Curvature
        int i;
        for (i = 0; i < 5; i++) {
            if (k >= curvatures[i + 1])
                break;
        }
        //System.out.println("i: " + i + ", r:" + r + ", k: " + k + ", (1.0/r): " + (1.0/r));
        double speedRange = speeds[i + 1] - speeds[i];
        double curvatureRange = curvatures[i] - curvatures[i + 1];
        double proportion = (curvatures[i] - k) / curvatureRange;
        double spd = speeds[i] + (proportion * speedRange);
        return Misc.clamp(spd, speeds[0], speeds[5]);
    }

    public static Vector2 carFrontCornerOffset2D(boolean right) {
        return new Vector2(octaneHalfLength + octaneLengthoffset, (right ? -1.0 : 1.0) * octaneHalfWidth);
    }
    public static Vector2 carFrontCorner(Vector2 cPf, Vector2 cOf, boolean right) {
        return cPf.plus(carFrontCornerOffset2D(right).rotateBy(cOf)); // Rotate and add offset
    }
    public static Vector2 carFrontCorner(CarData car, boolean right) { //Assumes on ground
        return carFrontCorner(car.position.flatten(), car.orientation.noseVector.flatten(), right);
    }
    public static Vector2 carDestForFrontCornerContact(Vector2 strikeLoc, Vector2 approachDirection, boolean right) {
        return strikeLoc.minus(carFrontCornerOffset2D(right).rotateBy(approachDirection)); // Rotate and add offset
    }

    public static double ballRadiusAtHeight2D(double height) { // Absolute height from bottom of ball; height clamps to [0,2*92.75]
        double fromCenterNormalized = clamp((height - ballR) / ballR, -1, 1);
        return Math.cos(Math.asin(fromCenterNormalized)) * ballR;
    }
    public static double ballContactRForPZ(double ballPZ) { // For car on ground
        double contactZFromBallBottom = Math.max(octaneFrontTopHeight - (ballPZ - Misc.ballR), 0);
        return clamp(ballRadiusAtHeight2D(contactZFromBallBottom), 0, ballContactR);
    }

    public static boolean withinCarXRange(DataPacket dp, double x) {
        Vector2[] jointToCarSides = new Vector2[4];
        jointToCarSides[0] = dp.cOf.scaledToMagnitude(Misc.octaneHalfLength + Misc.octaneLengthoffset);
        jointToCarSides[1] = dp.cOf.scaledToMagnitude(Misc.octaneHalfLength - Misc.octaneLengthoffset).rotateBy(Math.PI);
        Vector2 carToSide = dp.cOf.scaledToMagnitude(Misc.octaneHalfWidth);
        jointToCarSides[2] = carToSide.rotateBy(Math.PI / 2.0);
        jointToCarSides[3] = carToSide.rotateBy(-Math.PI / 2.0);
        double positiveXPart = 0, negativeXPart = 0; // Magnitudes, two contributions each (both values are >=0)
        int posCount = 0, negCount = 0;
        for(Vector2 part : jointToCarSides) {
            if(part.x != 0 ? part.x > 0 : part.y > 0) { // Y to split up sides of vector at 0 or PI degrees
                positiveXPart += part.x;
                posCount++;
            } else {
                negativeXPart -= part.x;
                negCount++;
            }
        }
        if(posCount != 2 || negCount != 2)
            System.out.println("[Misc.withinCarXRange] posCount or negCount not equal to 2! posCount: " + posCount + ", negCount: " + negCount);
        return x > dp.cP.x - negativeXPart && x < dp.cP.x + positiveXPart;
    }

    public static Vector2 wallShotBounceLoc(Vector2 ballStart, double wallXSgn, int team) { // wallXSgn default + (1)
        double bX = ballStart.x;
        double bY = ballStart.y;
        double nX = targetNet(team).x;
        double nY = targetNet(team).y;
        //double wX = (wallX - ballR) * (bX > sgnNo0(wallXSgn) ? 1.0 : -1.0); // Wall X position (minus ball radius); x pos of ball on wall contact
        double wX = (wallX - ballR) * sgn(wallXSgn); // Wall X position (minus ball radius); x pos of ball on wall contact
        double wY = ((bX*nY) + (nX*bY) - (bY*wX) - (nY*wX)) / (bX + nX - (2.0*wX)); // Wall Y position; y pos of ball on wall contact
        return new Vector2(wX, wY);
    }



    public static Boolean matchIsUnlimitedBoost;
    public static boolean unlimitedBoost() {
        if(matchIsUnlimitedBoost == null) {
            long startT = System.nanoTime();
            rlbot.flat.MatchSettings MS = null;
            try {
                MS = RLBotDll.getMatchSettings();
            } catch(Exception e) {
                e.printStackTrace();
            }

            if (MS == null)
                System.out.println("[Misc.lineDriveDuration] RLBotDll.getMatchSettings() null");
            else
                matchIsUnlimitedBoost = MS.mutatorSettings().boostOption() == BoostOption.Unlimited_Boost;
            System.out.println("[Misc.unlimitedBoost] Setting matchIsUnlimitedBoost took " + ((System.nanoTime() - startT) / 1000000.0) + "ms");
        }
        return matchIsUnlimitedBoost;
    }
    public static double lineDriveDuration(double vi, double d, double boostAllotted, double maxEndV) {
        //long t1 = System.nanoTime();
        double dTarget = Math.abs(d);
        double dt = 1.0 / 120.0; // Physics frame time (120Hz period)
        double dt2 = 1.0 / 14400.0; // dt squared
        final double boostUsageRate = 33.3;
        double throttle = 1;
        final boolean unlimitedBoost = unlimitedBoost();

        double dCurr = 0, vCurr = vi, aCurr = 0;
        double boostRemaining = clamp(boostAllotted, 0, 100);
        int frames = 0;

        while(dCurr < dTarget) {
            dCurr += (vCurr * dt) + 0.5 * (aCurr * dt2); // d_new = d + vt + 0.5at^2
            aCurr = 0;
            boolean boosting = false;
            double distUntilBrake = d - dCurr - brakeDist(vCurr, maxEndV) - (vCurr * dt);

            if(maxEndV != -1 && vCurr > maxEndV && distUntilBrake < 0) { //Brake
                aCurr = -3500; // Brake acceleration
            } else {
                boosting = (boostRemaining > 0 || unlimitedBoost) && vCurr < 2300; // Boost if possible
                aCurr = throttle * throttleAccelAtSpd(vCurr);
            }
            if(boosting) {
                aCurr += 991.667; // Boost acceleration
                boostRemaining -= boostUsageRate * dt;
            }

            double vCurrMax = boosting ? 2300 : Math.max(vCurr, 1410); // If (at max speed) or (faster than 1410 and not boosting), can't speed up
            vCurr = Math.min(vCurr + (aCurr * dt), vCurrMax); // v = v + at
            frames++;
            if(frames > 10000 || (vCurr <= 0 && aCurr <= 0))
                break;
        }
        //System.out.println("lDD loop took " + ((System.nanoTime() - t1) / 1000000.0) + "ms");

        double timeElapsed = frames * dt;
        //double vFinal = vCurr; # This could also be returned
        return timeElapsed; // Could return boostRemaining too
    }
    public static double lineDriveDuration(double vi, double d, double boostAllotted) {
        return lineDriveDuration(vi, d, boostAllotted, -1);
    }

    // Distance needed to change velocities (initial (vi) -> final (vf)) using acceleration a while driving straight
    public static double accelDist(double vi, double vf, double a) { // vf^2 = vi^2 + 2*a*d  ->  d=(vf^2 - vi^2)/(2*a)
        return (Math.pow(vf, 2) - Math.pow(vi, 2)) / (2.0 * a);
    }
    public static double coastDist(double vi, double vf){
        return accelDist(vi, vf, -525.0); // Coast accel -525.0
    }
    public static double coastDist(double vi) {
        return coastDist(vi, 0);
    }
    public static double brakeDist(double vi, double vf){
        return accelDist(vi, vf, -3500.0); // Brake accel -3500.0
    }
    public static double brakeDist(double vi) {
        return brakeDist(vi, 0);
    }

    public static double steerForErr(double headingErrRad) { // Not clamped
        return -Misc.sgn(headingErrRad) * (10.0 * (Math.pow(Math.abs(headingErrRad), 1.2)));
    }
    public static double steerForErr(double headingErrRad, double lastHeadingErrRad, double dt) { //TODO: Incorporate car speed (via turnRadiusForSpeed?)
        if(dt <= 0)
            return 0;
        double P = steerForErr(headingErrRad);
        double changeDeg = Math.toDegrees(headingErrRad - lastHeadingErrRad) / dt; //5deg in perfect frame => 300
        double D = -changeDeg / 300.0;
        return Misc.clamp(P + D, -1, 1);
    }

    public static ControlsOutput spinLineDrive(DataPacket dp, Vector2 dest, double timeAllotted, double maxEndV) {
        double carSpd = dp.cForwardVf;
        if(Math.abs(dest.x) > 800 && Math.abs(dp.cPf.x) < 885 && Math.abs(dp.cPf.y) > 5050) // Escape net
            dest = new Vector2(800 * sgn(dest.x), dest.y);

        Vector2 carToDest = dest.minus(dp.cPf);
        double straightLineDist = carToDest.magnitude();
        double headingErr = dp.cOf.angleTo(carToDest); // Radians
        double headingErrDegAbs = Math.abs(Math.toDegrees(headingErr));

        ControlsOutput cO = new ControlsOutput().withSteer(steerForErr(headingErr));

        double carSpdPerFrame = carSpd * frameT;
        double distUntilBrake = straightLineDist - brakeDist(carSpd, maxEndV) - carSpdPerFrame;
        double distUntilBrakeDelayed = distUntilBrake - carSpdPerFrame;
        double spareTime = timeAllotted < 0 ? -100 : timeAllotted - lineDriveDuration(carSpd, straightLineDist, dp.car.boost, maxEndV);

        double throttle;
        boolean boost = false, slide = false;
        if(maxEndV == 0 && ((straightLineDist < 10 && headingErrDegAbs > 1) || straightLineDist < 5)) {     // Parking
            cO.withSteer(0);
            if(Math.abs(carSpd) > 100)          // Slow down
                throttle = -sgn(carSpd);
            else if(straightLineDist < 1)       // Chill out
                throttle = 0;
            else if(straightLineDist < 5)       // Close enough, hold position
                throttle = 0.3 * -sgn(carSpd);
            else                                // Get closer
                throttle = -sgn(headingErrDegAbs - 90);
        } else if(maxEndV != -1 && carSpd > maxEndV && (distUntilBrakeDelayed < 0 || distUntilBrake < 0)) { // Braking
            throttle = -1;
        } else if(headingErrDegAbs < 5) { // Facing target
            if(spareTime < 0) { // Just go
                throttle = 1;
                boost = true;
            } else if(spareTime < 0.1) { // Coast
                throttle = 0;
            } else {
                double targetSpd = 300; //TODO: Fix for close range?
                throttle = -sgn(carSpd - targetSpd);
            }
        } else { // Turn to target
            throttle = 0.7;
            if(false && spareTime > 0.1) //TODO; Reimplement?
                slide = headingErrDegAbs > 50;
            else
                slide = carSpd > 400 && headingErrDegAbs > 100;
        }

        boolean tooLateForBoost = distUntilBrakeDelayed < 200 && maxEndV != -1 && carSpd > maxEndV;
        boost = boost && !slide && dp.cVfMag < 2297 && headingErrDegAbs < 5 && !tooLateForBoost;
        cO = cO.withThrottle(throttle).withBoost(boost).withHandbrake(slide);
        String debugText = String.format("[Misc.spinLineDrive] throttle: %.3f, steer: %.3f; boost? %b, slide? %b; spareTime: %.3f, d: %.1f, hErr: %.1f",
                cO.getThrottle(), cO.getSteer(), cO.holdBoost(), cO.holdHandbrake(), spareTime, straightLineDist, Math.toDegrees(headingErr));
        //System.out.println(debugText);
        return cO;
    }

    public static boolean ballAboveCar(DataPacket dp) {
        double ballContactRAdjusted = ballContactR - 2;
        boolean ballPYOK = dp.cRelY < octaneHalfLength - octaneLengthoffset + ballContactRAdjusted && dp.cRelY > -(octaneHalfLength + octaneLengthoffset + ballContactRAdjusted);
        boolean ballPXOK = Math.abs(dp.cRelX) < octaneHalfWidth + ballContactRAdjusted;
        return ballPYOK && ballPXOK;
    }
    public static boolean ballCenterAboveCar(DataPacket dp) {
        boolean ballPYOK = dp.cRelY < Misc.octaneHalfLength - Misc.octaneLengthoffset && dp.cRelY > -(Misc.octaneHalfLength + Misc.octaneLengthoffset);
        boolean ballPXOK = Math.abs(dp.cRelX) < Misc.octaneHalfWidth;
        return ballPYOK && ballPXOK;
    }
    public static boolean dribbleAnglesOK(Vector2 cOf, Vector2 bVf) {
        double headingToBallV = Math.abs(Math.toDegrees(cOf.angleTo(bVf)));
        return bVf.magnitude() < 100 || headingToBallV < 90;//35
    }
    public static boolean ballOnCar(DataPacket dp) {
        if(!ballAboveCar(dp))
            return false;
        boolean ballZOK = dp.bP.z > Misc.ballR + 2 && dp.bP.z < octaneFrontTopHeight + Misc.ballR + (dp.car.position.z - octaneJointHeight) + 20; //5
        boolean ballVOK = Math.abs(dp.bV.z) < 300;
        //System.out.println("ballZOK? " + ballZOK + ", ballVOK? " + ballVOK + ", anglesOK? " + anglesOK);
        return ballZOK && ballVOK;// && dribbleAnglesOK(dp.cOf, dp.bVf);
    }

    public static Vector2 dodgeDirection(Vector2 absoluteDodgeDirection, Vector2 carNoseOrientation) {
        return absoluteDodgeDirection.rotateBy((Math.PI / 2.0) - carNoseOrientation.angle());
    }

    // Intersection of line between points linePoint1 and linePoint2 and ray from point rayStart in direction rayDirection
    public static Vector2 rayLineIntersect(Vector2 linePoint1, Vector2 linePoint2, Vector2 rayStart, Vector2 rayDirection) {
        rayDirection = rayDirection.normalized();
        Vector2 v1 = rayStart.minus(linePoint1);
        Vector2 v2 = linePoint2.minus(linePoint1);
        Vector2 v3 = new Vector2(-rayDirection.y, rayDirection.x);
        double dot = v2.dotProduct(v3);
        if (Math.abs(dot) < 0.000001)
            return null;

        // a x b == a.cross(b) == "Cross-product" of 2 2D vectors: S implifies to (a.x * b.y - a.y * b.x)
        double t1 = (v2.x * v1.y - v2.y * v1.x) / dot;
        double t2 = v1.dotProduct(v3) / dot;
        if (t1 >= 0.0 && (t2 >= 0.0 && t2 <= 1.0))
            return rayStart.plus(rayDirection.scaled(t1));
        return null;
    }

    //Returns the center of the opening of the net requested
    public static Vector3 targetNet(int team) {
        return (team == 1 ? blueNetPos : orangeNetPos);
    }
    public static Vector3 ownNet(int team) {
        return (team == 1 ? orangeNetPos : blueNetPos);
    }
    public static double netWallXIntercept(double targetNetY, Vector2 ballPf, Vector2 ballVf) {
        return ballPf.x + ((targetNetY - ballPf.y) / (ballVf.y / ballVf.x)); //Solved from point-slope form for line (slope from ballVf, pos from ballP)
    }
    public static boolean ballTowardsNet(Vector3 bV, Vector2 bPf, int team, double minDistFromPostsAndWall, boolean ownNet) {
        double netY = Misc.netY * (team == (ownNet ? 0 : 1) ? -1.0 : 1.0);

        if(Misc.sgn(bV.y) * Misc.sgn(netY) < 0) // Ball going towards other half
            return false;
        double netWallXIntercept = Misc.netWallXIntercept(netY, bPf, bV.flatten());
        return Math.abs(netWallXIntercept) < Misc.netHalfWidth - Misc.ballR - minDistFromPostsAndWall;
    }
    public static boolean ballTowardsNet(DataPacket dp, double minDistFromPostsAndWall, boolean ownNet) {
        return ballTowardsNet(dp.bV, dp.bPf, dp.team, minDistFromPostsAndWall, ownNet);
    }
    public static boolean ballTowardsNet(DataPacket dp, boolean ownNet) {
        return ballTowardsNet(dp, 0, true);
    }
    public static Vector3 bestTargetNetLoc(Vector3 ballP, Vector2 ballVf, double minDistFromPostsAndWall, int team) {
        Vector2 ballPf = ballP.flatten();
        Vector3 tNP = targetNet(team);

        double targetX;
        if(ballVf.x == 0 || ballVf.y == 0) //Ball not moving
            targetX = ballP.x;
        else if(Misc.sgn(ballVf.y) == Misc.sgn(tNP.y)) { //Ball moving towards their half
            targetX = netWallXIntercept(tNP.y, ballPf, ballVf);
        } else //Ball moving towards our half
            targetX = Misc.sgn(ballVf.x) * netHalfWidth; //Clamp will do the rest

        double distInwards = ballR + minDistFromPostsAndWall;
        double targetNetX = Misc.clamp(targetX, tNP.x - netHalfWidth + distInwards, tNP.x + netHalfWidth - distInwards);
        double targetNetZ = Misc.clamp(ballP.z, ballR, tNP.z + (netHeight / 2.0) - distInwards);
        return new Vector3(targetNetX, tNP.y, targetNetZ);
    }
    public static Vector3 bestTargetNetLoc(Vector3 ballP, Vector2 ballVf, int team) {
        return bestTargetNetLoc(ballP, ballVf, 0, team);
    }

    public static boolean ballInFrontOfNet(Vector2 bPf, double minDistFromPostsAndWall) { // Either net
        return Math.abs(Misc.netY - Math.abs(bPf.y)) < 150 && Math.abs(bPf.x) < Misc.netHalfWidth - Misc.ballR - minDistFromPostsAndWall;
    }
    public static boolean ballInFrontOfNet(Vector2 bPf) {
        return ballInFrontOfNet(bPf, 0);
    }
}
