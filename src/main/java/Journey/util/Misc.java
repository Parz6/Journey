package Journey.util;

import Journey.input.CarData;
import Journey.input.DataPacket;
import Journey.output.ControlsOutput;
import Journey.vector.Vector2;
import Journey.vector.Vector3;
import rlbot.cppinterop.RLBotDll;
import rlbot.flat.BoostOption;

public class Misc {
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

    // Returns val constrained between min and max (or +-bound)
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    public static double clamp(double val, double bound) {
        return bound > 0 ? clamp(val, -bound, bound) : clamp(val, bound, -bound);
    }

    public static double throttleAccelAtSpd(double s) { //Full (1.0) throttle accel at speed s
        if (s == 0)
            return 1600;
        s = Math.abs(s);
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

    // Acute positive angle (radians) from chord (start to end) to tangent of start point for chord length x across circle of radius r
    public static double acuteHeadingOffsetForArcTurn(Vector2 start, Vector2 end, double r) { // Acute heading offset from start -> end aka end.minus(start)
        double x = start.distance(end);
        r = Math.max(x / 2.0, r); // Forces x: [0, 2r]; r < x/2 is an impossible situation (chord longer than diameter)
        return Math.asin(x / (2.0 * r)); // arcsin(x/2r): [0, pi/2] for x: [0, 2r]
    }
    // It doesn't matter what the start orientation is; there's only one radius given start and end positions and ending orientation
    public static double radiusForArcTurn(Vector2 startPf, Vector2 endPf, Vector2 endOf) {
        Vector2 displacement = endPf.minus(startPf); //Displacement between start and end points
        double distance = displacement.magnitude(); //Distance d between start and end points
        double theta1 = displacement.angleTo(endOf); //Shot angle
        //double theta = 2.0 * theta1; //Arc angle
        //while (theta < 0)
        //    theta += Math.PI * 2.0;
        double radius = distance / (2.0 * Math.sin(theta1)); //d / (2.0 * Math.sin(theta))
        return Math.abs(radius);
    }
    public static double arcTurnDuration(double radius, double thetaRad, double v) {
        return thetaRad * radius / v; // ((thetaRad / 2*pi) * (2*pi*r)) / v
    }
    public static double arcTurnDuration(double radius, double thetaRad) {
        return arcTurnDuration(radius, thetaRad, speedForTurnRadius(radius));
    }
    public static double avgSpeedForArcTurnTime(double distance, double radius, double timeAllotted) {
        double arcLen = 2.0 * radius * Math.asin(distance / (2.0 * radius));
        return arcLen / timeAllotted;
    }

    public static Vector2 carFrontCornerOffset2D(boolean right) {
        return new Vector2(octaneHalfLength + octaneLengthoffset, (right ? -1.0 : 1.0) * octaneHalfWidth);
    }
    public static Vector2 carFrontCorner(CarData car, boolean right) { //Assumes on ground
        return car.position.flatten().plus(carFrontCornerOffset2D(right).rotateBy(car.orientation.noseVector.flatten())); // Rotate and add offset
    }
    public static Vector2 carDestForFrontCornerContact(Vector2 strikeLoc, Vector2 approachDirection, boolean right) {
        return strikeLoc.minus(carFrontCornerOffset2D(right).rotateBy(approachDirection)); // Rotate and add offset
    }

    // Returns: {arcEnd to locationArcEndFaces, arc % of circle as an angle} OR null if impossible (radius too big)
    public static Vector2[] vectorToArcEnd(Vector2 arcStart, Vector2 locationArcStartFaces, Vector2 locationArcEndFaces, double arcRadius) { //Vector from arc end to locationArcEndFaces
        Vector2 straightawayToCar = locationArcEndFaces.minus(arcStart); //Originally carLocRel
        Vector2 strikeToStraightaway = arcStart.minus(locationArcStartFaces);
        double rotatorRad = strikeToStraightaway.angleTo(straightawayToCar);
        Vector2 carLocRelRotated = (new Vector2(0, straightawayToCar.magnitude())).rotateBy(Math.abs(rotatorRad));
        boolean arcToRight = rotatorRad < 0;
        Vector2 finalCarLoc = carLocRelRotated.plus(new Vector2(arcRadius, 0));
        Vector2[] absoluteResultArr = absoluteVectorToArcEnd(finalCarLoc, arcRadius);
        if (absoluteResultArr == null) //Impossible with given turn radius
            return null;
        Vector2[] absoluteResult = absoluteResultArr;
        if (arcToRight) {
            absoluteResult[0] = new Vector2(-absoluteResult[0].x, absoluteResult[0].y); //Un-mirror over y
            absoluteResult[1] = (new Vector2(absoluteResult[1].x, -absoluteResult[1].y)).normalized(); //TODO: Is this right? Should it not be done at all?
        }
        Vector2[] result = new Vector2[2];
        result[0] = absoluteResult[0].rotateBy(strikeToStraightaway.angle() - (Math.PI / 2.0)); //Result (rotated back)
        result[1] = absoluteResultArr[1]; //Theta (of arc) as vector
        return result;
    }
    // Assuming arc of radius starts at (radius, 0) facing 90deg (positive y) and ends pointing towards target; Null if impossible (radius too big)
    private static Vector2[] absoluteVectorToArcEnd(Vector2 target, double radius) { // Target to arc end
        double x = target.x;
        double y = target.y;
        double radical = Math.sqrt(-Math.pow(radius, 2) + Math.pow(x, 2) + Math.pow(y, 2));
        double thetaNeg = 2.0 * Math.atan((y - radical) / (x + radius)); //+radical instead for clockwise (wrong way)
        //double slope = -1.0 / Math.tan(thetaNeg);
        if (Double.isNaN(thetaNeg)) //Impossible with given turn radius
            return null;
        Vector2 thetaLocVec = new Vector2(radius * Math.cos(thetaNeg), radius * Math.sin(thetaNeg));
        Vector2[] result = new Vector2[2];
        result[0] = thetaLocVec.minus(target); //Result
        result[1] = new Vector2(Math.cos(thetaNeg), Math.sin(thetaNeg)); //Theta as vector
        return result;
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
            if(vCurr < 0 && aCurr < 0)
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
    public static Vector3 bestTargetNetLoc(Vector3 ballP, Vector2 ballVf, double minDistFromPostsAndWall, int team) {
        Vector2 ballPf = ballP.flatten();
        Vector3 tNP = targetNet(team);

        double targetX = 0;
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
}
