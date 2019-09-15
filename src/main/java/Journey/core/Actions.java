package Journey.core;

import Journey.Journey;
import Journey.boost.BoostManager;
import Journey.boost.BoostPad;
import Journey.input.CarData;
import Journey.input.DataPacket;
import Journey.output.ControlsOutput;
import Journey.prediction.BallPredictionHelper;
import Journey.util.ArcMath;
import Journey.util.Misc;
import Journey.util.Rendering;
import Journey.vector.Vector2;
import Journey.vector.Vector3;
import rlbot.render.Renderer;

import java.awt.Color;

import static Journey.util.Misc.sgn;

public class Actions {
    private Journey bot;
    private boolean dodgeLock = false, flickLock = false; // When true, the corresponding action is taken

    public Actions(Journey bot) {
        this.bot = bot;
    }

    public ControlsOutput runBotCore(DataPacket dp, Renderer renderer) {
        ControlsOutput cO;
        BallPredictionHelper.BPslice fCS = null;

        dribbling = Misc.ballAboveCar(dp);//TODO: Change? Misc.ballOnCar(dp);//Misc.ballCenterAboveCar(dp);
        if(!dribbling)
            fCS = firstCatchableSlice(dp, 1.5);
        if(dribbling || catching || (fCS != null && shouldCatch(dp, fCS))) {
            cO = dribble(dp, fCS);
        } else {
            cO = chasePushFRS(dp, renderer);//new ControlsOutput();//
        }
        return cO;
    }


    /*
        Checks and decision making
     */
    public boolean actionLockExists() {
        return dodgeLock || flickLock;
    }
    public static boolean shouldEscapeWall(DataPacket dp) {
        return dp.cP.z > 100 && dp.car.hasWheelContact && Math.abs(dp.cP.x) > Misc.netHalfWidth && dp.cO.z > -0.6;
    }
    public static boolean shouldPushBallIn(DataPacket dp) {
        Vector3 netLoc = Misc.targetNet(dp.team);
        Vector2 bestNetLoc = Misc.bestTargetNetLoc(dp.bP, dp.bVf, dp.team).flatten();
        double ballGoalWallXInt = Misc.netWallXIntercept(netLoc.y, dp.bPf, dp.bVf);
        final double ballTargetXTolerance = -100; //200 original
        boolean ballRollingIn = (Math.abs(ballGoalWallXInt) < (Misc.netHalfWidth - Misc.ballR + ballTargetXTolerance)) && (Misc.sgn(dp.bV.y) == Misc.sgn(netLoc.y));
        if(!ballRollingIn)
            return false;

        double distBetween = dp.cPf.distance(dp.bPf) - Misc.octaneHalfLength - Misc.octaneLengthoffset - Misc.ballContactR;
        //final double posToleranceDeg = 15.0 + Misc.clamp(15.0 * (1000.0 - distBetween) / 1000.0, 0, 15.0); //OG
        final double posToleranceDeg = 20.0 + Misc.clamp(15.0 * (1000.0 - distBetween) / 1000.0, 0, 15.0);
        //final double oriToleranceDeg = 60.0 - Misc.clamp(35.0 * (1000.0 - distBetween) / 1000.0, 0, 35.0); //OG
        final double oriToleranceDeg = 90.0 - Misc.clamp(35.0 * (1000.0 - distBetween) / 1000.0, 0, 35.0);

        //boolean ballSlow = dp.bVf.magnitude() < 300;
        boolean carBehindBall1 = Math.abs(dp.bPf.minus(dp.cPf).angleTo(netLoc.minus(dp.bP).flatten())) < Math.toRadians(posToleranceDeg);
        boolean carBehindBall2 = Math.abs(dp.bPf.minus(dp.cPf).angleTo(bestNetLoc.minus(dp.bPf))) < Math.toRadians(posToleranceDeg);
        boolean carBehindBall = carBehindBall1 || carBehindBall2;
        boolean carFacingBall = Math.abs(dp.cOf.angleTo(dp.bPf.minus(dp.cPf))) < Math.toRadians(oriToleranceDeg);
        boolean ballLow = dp.bP.z < Misc.octaneFrontTopHeight + Misc.ballR;
        //boolean distOK = distBetween > -10;// && distBetween < 700; //OG: distBetween < 500 //10 units overlapping car if facing ball TODO: Adjust/remove?
        //System.out.println("[Misc.shouldPushBallIn] ballRollingIn=" + ballRollingIn + ", ballSlow=" + ballSlow + ", carBehindBall=" + carBehindBall + ", carFacingBall=" + carFacingBall + ", ballLow=" + ballLow + ", distOK=" + distOK);
        return carBehindBall && carFacingBall && ballLow;
    }
    public static boolean shouldGetBoost(DataPacket dp, BoostPad bP) {
        double carToBallApprox = dp.cPf.distance(dp.bPf) - Misc.ballContactR - Misc.octaneHalfLength - Misc.octaneLengthoffset;
        return dp.car.boost < 40 && carToBallApprox > 75 && carToBallApprox > dp.cPf.distance(bP.targetPickupLoc(dp.cPf)); //TODO: Add?
    }
    public static boolean shouldCatch(DataPacket dp, BallPredictionHelper.BPslice fCS) { // TODO: Expand?
        boolean anglesOK = Misc.dribbleAnglesOK(fCS.p.flatten().minus(dp.cPf), fCS.v.flatten());
        return anglesOK;
    }


    /*
        Basic actions
     */
    public static ControlsOutput kickoff(DataPacket dp) {
        ControlsOutput cO = new ControlsOutput();
        Vector3 locBehindBall = dp.bP.plus(dp.bP.minus(Misc.targetNet(dp.team)).normalized().scaledToMagnitude(Misc.ballR + 100.0));
        // Yes, this dist value is basically meaningless
        double dist = dp.cPf.distance(locBehindBall.flatten()) - Misc.octaneHalfLength; // Diagonal starts at 3071, off-center at 3597, straight at 4300
        boolean jump = (dist > 280 && dist < (Math.abs(dp.cP.x) < 25 ? 500 : 400)) || (dist > (Math.abs(dp.cP.x) < 25 ? 3570 : 3090) && dist < (Math.abs(dp.cP.x) < 25 ? 3700 : 3200));
        boolean flip1 = dist > (Math.abs(dp.cP.x) < 25 ? 3300 : 2800) && dist < (Math.abs(dp.cP.x) < 25 ? 3470 : 2970);
        boolean flip2 = dist > 0 && dist < (Math.abs(dp.cP.x) < 25 ? 90 : 180);
        if(jump && Math.abs(dp.cP.x) < 1200) { // Jump
            System.out.println("[Journey.kickoff] Kickoff (jumping)");
            cO = new ControlsOutput().withJump(true);
        } else if((flip1 || flip2) && !dp.car.hasWheelContact) { // Flip
            if(flip1) {
                System.out.println("[Journey.kickoff] Kickoff (flip 1)");
                cO = new ControlsOutput().withJump(true).withPitch(-1);
            } else {
                System.out.println("[Journey.kickoff] Kickoff (flip 2); x: " + dp.cP.x);
                double yawOutput = -Misc.sgn(dp.cP.y) * dp.cP.x / 125.0;
                cO = new ControlsOutput().withJump(true).withPitch(-1).withYaw(yawOutput);
            }
        } else { // Goto locBehindBall
            double headingErr = dp.cOf.angleTo(locBehindBall.flatten().minus(dp.cPf)); // Use predicted ballPf for a calculated intercept location
            cO.withSteer(Misc.steerForErr(headingErr));
        }
        boolean carNoseZOK = dp.cO.z > -0.6 && dp.cO.z < 0.2;
        cO.withBoost(dp.cV.magnitude() < 2297 && (dist > 3850 || (carNoseZOK && dp.car.orientation.roofVector.z > 0)));
        return cO.withThrottle(1);
    }
    public ControlsOutput airRecovery(DataPacket dp, Renderer renderer) {
        System.out.println("[Journey.airRecovery] Recovering");
        if(renderer != null)
            renderer.drawString3d("Recovering", Color.magenta, dp.cP, 2, 2);
        ControlsOutput cO = new ControlsOutput();
        Vector3 rightO = dp.car.orientation.rightVector;
        Vector3 roofO = dp.car.orientation.roofVector;
        cO.withRoll(rightO.z);
        cO.withPitch(-dp.cO.z);
        cO.withJump(bot.framesElapsed % 2 == 0 && dp.cP.z < 80 && roofO.z < 0.2 && !dp.car.hasWheelContact); // Every other frame
        return cO;
    }
    public static ControlsOutput escapeNet(DataPacket dp, Renderer renderer) {
        System.out.println("[Journey.escapeNet] Escaping net");
        renderer.drawString3d("Escaping net", Color.magenta, dp.cP, 2, 2);
        Vector2 rayLineInt = Misc.rayLineIntersect(new Vector2(-5500, Misc.netY * sgn(dp.cP.y)), new Vector2(5500, Misc.netY * sgn(dp.cP.y)), dp.cPf, dp.bPf.minus(dp.cPf));
        Vector2 dest;
        if(rayLineInt != null)
            dest = new Vector2(Misc.clamp(rayLineInt.x, Misc.netHalfWidth - 90), (Misc.netY - 20) * sgn(dp.cP.y));
        else
            dest = new Vector2(Misc.clamp(dp.bP.x, Misc.netHalfWidth - 100), Misc.netY * sgn(dp.cP.y));
        if(renderer != null) {
            renderer.drawCenteredRectangle3d(Color.red, dest.inflate(50), 15, 15, true);
            renderer.drawLine3d(Color.red, dp.cPf.inflate(50), dest.inflate(50));
        }
        return Misc.spinLineDrive(dp, dest, -1, -1).withBoost(false);
    }
    public static ControlsOutput escapeWall(DataPacket dp, Renderer renderer) {
        System.out.println("[Journey.escapeWall] Escaping wall");
        if(renderer != null)
            renderer.drawString3d("Escaping wall", Color.magenta, dp.cP, 2, 2);
        double goalWallDist = Math.abs(Math.abs(dp.cP.y) - 4909);
        double sideWallDist = Math.abs(Math.abs(dp.cP.x) - 3790);
        double steer;
        if(sideWallDist < goalWallDist) { // On one of the side walls
            steer = -sgn(dp.cO.y) * sgn(dp.cP.x);
        } else { // On one of the goal walls
            steer = sgn(dp.cO.x) * sgn(dp.cP.y);
        }
        return new ControlsOutput().withSteer(steer).withThrottle(1).withHandbrake(dp.cO.z > 0.4);
    }
    public ControlsOutput pushBallIn(DataPacket dp, Renderer renderer) { // TODO: Improve, see rlbot/src/.../plans/PushToNetPlan
        Vector2 carToBall = dp.bPf.minus(dp.cPf);
        double headingToBall = Math.abs(Math.toDegrees(carToBall.angleTo(dp.cOf)));
        boolean headingToBallOKForDodge = headingToBall < 10;
        boolean speedsOKForDodge = dp.bVfMag > 1400 && dp.bVfMag < 2300 && dp.cVfMag < 2297;
        if(dp.newBallTouch && dp.ball.latestTouch.playerIndex == dp.playerIndex && headingToBallOKForDodge && speedsOKForDodge && dp.car.hasWheelContact) { // Dodge if just touched the ball
            System.out.println("[Journey.pushBallIn] Dodging into ball");
            startDodge(Misc.dodgeDirection(carToBall, dp.cOf), 0.05, 0.1);
            return dodge(dp);
        }
        System.out.println("[Journey.pushBallIn] Pushing ball to net");
        //Vector2 dest = dp.bPf.minus(Misc.targetNet(dp.team).minus(dp.bP).flatten().scaledToMagnitude(Misc.ballContactR));
        Vector2 dest = dp.bPf.plus(Misc.targetNet(dp.team).minus(dp.bP).flatten().scaledToMagnitude(20));
        if(renderer != null) {
            renderer.drawCenteredRectangle3d(Color.red, dest.inflate(50), 15, 15, true);
            renderer.drawLine3d(Color.red, dp.cPf.inflate(50), dest.inflate(50));
        }
        ControlsOutput cO = Misc.spinLineDrive(dp, dest, -100, 2300);

        boolean carNoseNextToBall = dp.cRelY > -(Misc.octaneHalfLength + Misc.octaneLengthoffset + 5);
        if(dp.cVfMag > 300 && (dp.cVfMag > dp.bVfMag || carNoseNextToBall) && (headingToBall > 5 || Math.abs(Math.toDegrees(carToBall.angleTo(dp.bVf))) > 2))
            cO.withThrottle(0).withBoost(false);
        return cO;
    }
    public static ControlsOutput getBoostPad(DataPacket dp, Renderer renderer, BoostPad bP) { //TODO: Finish
        System.out.println("[Journey.getBoostPad] Getting boost pad");
        Vector2 dest = bP.targetPickupLoc(dp.cPf);
        if(renderer != null) {
            renderer.drawCenteredRectangle3d(Color.red, dest.inflate(50), 15, 15, true);
            renderer.drawLine3d(Color.red, dp.cPf.inflate(50), dest.inflate(50));
        }
        return Misc.spinLineDrive(dp, dest, -1, BoostManager.boostPickupMaxEndV);
    }
    public ControlsOutput handleLocks(DataPacket dp) {
        if(dodgeLock)
            return dodge(dp);
        if(flickLock)
            return flick(dp);

        System.out.println("[Actions.handleLocks] No locks to handle!");
        return new ControlsOutput();
    }


    /*
        General dodge and pop
     */
    private Vector2 dodgeDir; // Right is (0,0), forward is (0,1)
    private double dodgeFlipT; // Desired flip delay (from initial jump start)
    private double dodgeHoldT; // Desired time to hold jump (initial)
    private double dodgeStartT, dodgeFinishT, dodgeWheelContactT;
    private int dodgeFlipFramesWaited; // Between initial jump and flip
    private boolean dodgeJumped; // Initial jump started
    private boolean dodgeIsPop; // Quick double jump instead of flip on second jump
    public void startDodge(Vector2 direction, double jumpHoldTime, double flipWaitTime, boolean pop) {
        System.out.println("STARTING DODGE; holdT: " + jumpHoldTime + ", flipT: " + flipWaitTime);
        dodgeDir = direction.isZero() ? new Vector2(0,1) : direction.normalized(); // Default forward
        dodgeFlipT = flipWaitTime;
        dodgeHoldT = jumpHoldTime;
        dodgeFlipFramesWaited = 0;
        dodgeJumped = false;
        dodgeLock = true;
        dodgeStartT = -1;
        dodgeFinishT = -1;
        dodgeIsPop = pop;
        dodgeWheelContactT = -1;
    }
    public void startDodge(Vector2 direction, double jumpHoldTime, double flipWaitTime) {
        startDodge(direction, jumpHoldTime, flipWaitTime, false);
    }
    public ControlsOutput dodge(DataPacket dp) {
        if(dodgeFinishT != -1) {
            if(dp.car.hasWheelContact && dodgeWheelContactT == -1)
                dodgeWheelContactT = dp.t;
            if(dodgeFinishT + Misc.postFlipMaxWaitTime < dp.t || (dodgeWheelContactT != -1 && dp.t - dodgeWheelContactT > Misc.postFlipWheelContactTimeout)) { // Wait for flip to complete
                dodgeLock = false;
            }
            return new ControlsOutput().withHandbrake();
        }

        if(dodgeStartT < 0)
            dodgeStartT = dp.t;
        double elapsed = dp.t - dodgeStartT;
        ControlsOutput cO = new ControlsOutput();

        if(dodgeIsPop) {
            if(elapsed > 0.5) {
                dodgeLock = false;
            } else if(dodgeJumped && dodgeFlipFramesWaited < Misc.minWaitFramesBeforeFlip) {
                System.out.print("DODGING[pop] (wait frame);");
                dodgeFlipFramesWaited++;
            } else {
                cO.withJump();
                if(dodgeJumped) {
                    boolean carBallYDiffOK = (dp.bP.y - dp.cP.y) * (dp.team == 0 ? 1.0 : -1.0) > 0; // Ball closer to target net than car
                    boolean ballBetweenCarAndTargetGoal = (Misc.withinCarXRange(dp, dp.bP.x - Misc.ballR) || Misc.withinCarXRange(dp, dp.bP.x + Misc.ballR));
                    if(carBallYDiffOK && ballBetweenCarAndTargetGoal) {
                        Vector2 carToBall = dp.bPf.minus(dp.cPf);
                        Vector2 popFlipDir = Misc.dodgeDirection(carToBall, dp.cOf);
                        cO.withPitch(-Math.sin(popFlipDir.angle()));
                        cO.withYaw(Math.cos(popFlipDir.angle()));
                        System.out.print("DODGING[pop] (flip!);");
                    } else
                        System.out.print("DODGING[pop] (second jump);");
                    dodgeFinishT = dp.t;
                } else
                    System.out.print("DODGING[pop] (initial jump);");
                dodgeJumped = true;
            }
            System.out.println(" " + Math.round(elapsed * 1000.0) + "ms elapsed");
            return cO;
        }

        if(dodgeJumped && elapsed > dodgeHoldT && dodgeFlipFramesWaited < Misc.minWaitFramesBeforeFlip) {
            System.out.print("DODGING (wait frame);");
            dodgeFlipFramesWaited++;
        } else if(elapsed > dodgeFlipT) {
            if(!dp.car.doubleJumped && dodgeFinishT == -1) {
                System.out.print("DODGING (flip!);");
                cO.withJump();
                cO.withPitch(-Math.sin(dodgeDir.angle()));
                cO.withYaw(Math.cos(dodgeDir.angle()));
                dodgeFinishT = dp.t;
            } else {
                System.out.print("DODGE TIMED OUT!");
                dodgeLock = false;
            }
        } else if(elapsed < dodgeHoldT || !dodgeJumped) {
            System.out.print("DODGING (initial jump);");
            cO.withJump();
            dodgeJumped = true;
        } else {
            System.out.print("DODGING (waiting);");
        }
        System.out.println(" " + Math.round(elapsed * 1000.0) + "ms elapsed");
        return cO;
    }


    /*
        Arc-based ball chase
     */
    private class Arc {
        public Vector2 endPf, endOf, bTNL, sL; // Ending location and orientation and target net loc
        public double r; //radius
        public boolean right;
        public Arc(Vector2 endPf, Vector2 endOf, double r, boolean right, Vector2 bTNL, Vector2 sL) {
            this.endOf = endOf;
            this.endPf = endPf;
            this.r = r;
            this.right = right;
            this.bTNL = bTNL;
            this.sL = sL;
        }
    }

    private BallPredictionHelper.BPslice cPFRS_lastFRS;
    private Arc cPFRS_lastArc;
    private static final double cPFRS_maxStrikeAngleDeg = 45; //40-50? carToStrikeLoc to strikeLocToBall degrees
    private static final double cPFRS_minTimeToDestForFlip = 1.5;//1.2?
    private static final double cPFRS_dodgeShotFlipInAdvanceT = Misc.frameT * 5.0;
    private static final double cPFRS_dodgeToContactT = 0.18;//0.15?
    public ControlsOutput chasePushFRS(DataPacket dp, Renderer renderer) { // "cPFRS"
        // Figure out the ball slice and target to use
        boolean reuseFRS = false;
        if(cPFRS_lastFRS != null) {
            boolean fRSNotStale = BallPredictionHelper.posErr(dp, cPFRS_lastFRS) < 10 && cPFRS_lastFRS.t > dp.t - 0.05; // was + 0.05 by accident
            boolean fRSReachable = dp.car.fRS(dp).t < cPFRS_lastFRS.t + 0.1;
            boolean newFRSMuchSooner = dp.car.fRS(dp).t < cPFRS_lastFRS.t - 0.5;
            reuseFRS = cPFRS_lastFRS.t < dp.t + 0.75 && fRSReachable && fRSNotStale && !newFRSMuchSooner && !dp.newBallTouch;
        }
        BallPredictionHelper.BPslice fRS;
        Arc arc;
        if(reuseFRS) { // Don't recalc FRS
            fRS = cPFRS_lastFRS;
            arc = cPFRS_lastArc;
        } else {
            fRS = dp.car.fRS(dp); // Assumes straight line
            Vector2 bestBallTarget = cPFRS_getBallTarget(dp, fRS);
            Vector2 fRSPf = fRS.p.flatten();
            Vector2 fRSToTarget = bestBallTarget.minus(fRSPf); //old dir: fRS.p.minus(dp.cP).flatten()
            Vector2 carToFRS = fRSPf.minus(dp.cPf);


            // Figure out where to go
            double velocitySLRotator = cPFRS_getVelocitySLRotator(fRS, bestBallTarget);
            Vector2 fRSToTargetVRotated = fRSToTarget.rotateBy(velocitySLRotator);
            double carToFRSToTargetVRotated = carToFRS.angleTo(fRSToTargetVRotated);
            //double basicShotAngle = carToFRS.angleTo(fRSToTarget); // >0 if arc <pi
            double basicShotAngle = carToFRS.angleTo(fRSToTargetVRotated);
            //boolean right = Math.abs(basicShotAngle) < Math.PI / 6.0 ? carToFRS.angleTo(dp.cOf) > 0 : basicShotAngle < 0;
            double carToBallToNetClosest = carToFRS.angleTo(new Vector2(0, bestBallTarget.y)); // Could also .angleTo(bestBallTarget.minus(fRSPf))
            boolean carToFRSInGoal = Math.abs(Misc.netWallXIntercept(dp.team == 0 ? Misc.netY : -Misc.netY, dp.cPf, fRSPf.minus(dp.cPf))) < Misc.netHalfWidth - 100;
            boolean targetIsNet = Math.abs(bestBallTarget.y) == Misc.netY;
            boolean right;
            System.out.println(carToBallToNetClosest);
            if(targetIsNet && Misc.ballInFrontOfNet(fRSPf)) {
                right = carToBallToNetClosest < 0;
            } else if(targetIsNet && carToFRSInGoal) {//if(Math.abs(carToFRS.angleTo(fRSToTarget)) < 0.03) { //0.03rad=1.72deg
                //right = carToFRS.angleTo(Misc.targetNet(dp.team).flatten().minus(fRSPf)) < 0;
                right = carToFRSToTargetVRotated < 0;
            } else {
                right = basicShotAngle < 0;
            }
            Vector2 currentCarCorner = Misc.carFrontCorner(dp.car, right);
            basicShotAngle = fRSPf.minus(currentCarCorner).angleTo(fRSToTargetVRotated);
            //basicShotAngle = basicShotAngle > 0 ? basicShotAngle : (Math.PI * 2.0) + basicShotAngle; // Convert the [-pi,pi] to [0,2pi]
            boolean ballTowardsOwnNet = Misc.ballTowardsNet(fRS.v, fRSPf, dp.team, 0, true);
            boolean ballTowardsOtherNet = Misc.ballTowardsNet(fRS.v, fRSPf, dp.team, 100, false);
            //double maxStrikeAngle = Math.toRadians(Misc.ballTowardsNet(dp, -500, false) ? 70 : cPFRS_maxStrikeAngleDeg);
            double maxStrikeAngle = ballTowardsOwnNet || ballTowardsOtherNet ? 80 : cPFRS_maxStrikeAngleDeg; //TODO: Combine with above line?
            double maxSLRotator = Math.max(0, Math.abs(basicShotAngle) - maxStrikeAngle); // StrikeLocation to ball to net, if straight line drive (abs)
            double initialDesiredArcAngle; // Max arc heading change (abs)
            /*
            if (maxSLRotator == 0) {
                //initialDesiredArcAngle = Math.PI / 6.0;
                initialDesiredArcAngle = 0;
                */
            if(Math.abs(basicShotAngle) < maxStrikeAngle) {
                initialDesiredArcAngle = Math.abs(basicShotAngle) / 2.0;
            } else {
                //initialDesiredArcAngle = Math.min(maxSLRotator, Math.PI / 3.0);
                initialDesiredArcAngle = maxSLRotator * 2.0;
            }
            //initialDesiredArcAngle = Math.toRadians(60);

            double initialArcAngle = Math.min(initialDesiredArcAngle, Math.max(0, dp.cOf.angleTo(fRSPf.minus(currentCarCorner)) * 2.0 * (right ? -1 : 1))); // Arc heading change (abs)
            double initialApproachAngle = -(basicShotAngle - (initialArcAngle / 2.0 * (right ? -1 : 1))); // An offset from fRSToTargetVRotated.angle()
            double initialSLRotator = -Math.max(0, Math.abs(initialApproachAngle) - maxStrikeAngle) * (right ? -1 : 1);
            double fRSContactRadius = Misc.ballContactRForPZ(fRS.p.z);
            Vector2 initialSLToFRS = fRSToTargetVRotated.scaledToMagnitude(fRSContactRadius).rotateBy(initialSLRotator);
            Vector2 initialSL = fRSPf.minus(initialSLToFRS);
            //System.out.println("[Actions.chasePushFRS] initialArcAngle 1: " + Math.round(Math.toDegrees(initialArcAngle)));
            //System.out.println("[Actions.chasePushFRS] basicShotAngle: " + Math.round(Math.toDegrees(basicShotAngle)) + ", initialApproachAngle: " + Math.round(Math.toDegrees(initialApproachAngle)));

            for (int i = 0; i < 5; i++) {
                basicShotAngle = initialSL.minus(currentCarCorner).angleTo(fRSToTargetVRotated);
                maxSLRotator = Math.max(0, Math.abs(basicShotAngle) - maxStrikeAngle); // StrikeLocation to ball to net, if straight line drive (abs)
                if(Math.abs(basicShotAngle) < maxStrikeAngle) {
                    initialDesiredArcAngle = Math.abs(basicShotAngle) / 2.0;
                } else {
                    initialDesiredArcAngle = maxSLRotator * 2.0;
                }
                //initialDesiredArcAngle = Math.toRadians(60);
                initialArcAngle = Math.min(initialDesiredArcAngle, Math.max(0, dp.cOf.angleTo(initialSL.minus(currentCarCorner)) * 2.0 * (right ? -1 : 1)));
                initialApproachAngle = -(basicShotAngle - (initialArcAngle / 2.0 * (right ? -1 : 1))); // An offset from fRSToTargetVRotated.angle()
                initialSLRotator = -Math.max(0, Math.abs(initialApproachAngle) - maxStrikeAngle) * (right ? -1 : 1);
                initialSLToFRS = fRSToTargetVRotated.scaledToMagnitude(fRSContactRadius).rotateBy(initialSLRotator);
                initialSL = fRSPf.minus(initialSLToFRS);
            }
            //System.out.println("[Actions.chasePushFRS] initialArcAngle 2: " + Math.round(Math.toDegrees(initialArcAngle)));
            //System.out.println("[Actions.chasePushFRS] basicShotAngle: " + Math.round(Math.toDegrees(basicShotAngle)) + ", initialApproachAngle: " + Math.round(Math.toDegrees(initialApproachAngle)));

            Vector2 initialApproachDir = fRSToTargetVRotated.rotateBy(initialApproachAngle).normalized();
            //Vector2 initialApproachDir = initialSL.minus(currentCarCorner).normalized();
            Vector2 initialCarDest = Misc.carDestForFrontCornerContact(initialSL, initialApproachDir, right);
            //Vector2 initialCOf = initialApproachDir.rotateBy(initialArcAngle * -sgn(basicShotAngle));
            //System.out.println("[Actions.chasePushFRS] " + Math.round(Math.toDegrees(initialArcAngle)));

            double initialTurnR = ArcMath.radiusForArcTurn(dp.cPf, initialCarDest, initialApproachDir);

            arc = new Arc(initialCarDest, initialApproachDir, initialTurnR, right, bestBallTarget, initialSL);
        }
        double initialAHOFAT = ArcMath.acuteHeadingOffsetForArcTurn(dp.cPf, arc.endPf, arc.r);
        Vector2 initialCarDest = arc.endPf;
        Vector2 initialApproachDir = arc.endOf;
        double initialTurnR = arc.r;
        boolean right = arc.right;
        Vector2 currentCarCorner = Misc.carFrontCorner(dp.car, right);
        Vector2 bTNL = arc.bTNL;
        Vector2 initialSL = arc.sL;
        Vector2 fRSPf = fRS.p.flatten();
        boolean ballTowardsOtherNet = Misc.ballTowardsNet(fRS.v, fRSPf, dp.team, 100, false);
        Vector2 initialCOf = initialApproachDir.rotateBy(initialAHOFAT * 2.0 * -(right ? -1 : 1));
        Vector2 carCorner = Misc.carFrontCorner(dp.cPf, initialCOf, right);

        if(renderer != null) {
            renderer.drawCenteredRectangle3d(Color.green, bTNL.inflate(fRS.p.z), 10, 10, true);                                         // Ball target
            renderer.drawCenteredRectangle3d(Color.green, fRS.p, 15, 15, true);                                                         // Ball
            renderer.drawLine3d(Color.green, bTNL.inflate(fRS.p.z), fRS.p);                                                                                 // Ball to ball target
            Rendering.drawImpact(renderer, initialSL.inflate(50), 40, Color.RED);                                                                 // Contact point
            renderer.drawLine3d(Color.red, fRSPf.inflate(50), initialSL.inflate(50));                                                                 // Ball to contact point
            renderer.drawCenteredRectangle3d(Color.red, carCorner.inflate(50), 15, 15, true);                                        // Desired car corner
            renderer.drawCenteredRectangle3d(Color.green, currentCarCorner.inflate(50), 8, 8, true);                                 // Current car corner
            Rendering.renderMinor2dArc(renderer, carCorner, initialSL, initialApproachDir, 50, Color.red, false);                         // Desired car corner to contact point
            renderer.drawLine3d(Color.blue, initialCarDest.inflate(50), initialCarDest.minus(initialApproachDir.scaledToMagnitude(200)).inflate(50)); // Approach angle
            renderer.drawLine3d(Color.blue, dp.cPf.inflate(50), dp.cPf.plus(initialCOf.scaledToMagnitude(200)).inflate(50));                          // Desired carOf
            renderer.drawLine3d(Color.green, dp.cPf.inflate(50), dp.cPf.plus(dp.cOf.scaledToMagnitude(100)).inflate(50));                             // Current carOf
            renderer.drawCenteredRectangle3d(Color.blue, initialCarDest.inflate(50), 15, 15, true);                                  // Car dest
            Rendering.renderMinor2dArc(renderer, dp.cPf, initialCarDest, initialApproachDir, 50, Color.blue, false);                      // Car to car dest
            Vector2 cornerToSLMidpoint = carCorner.plus(initialSL.minus(carCorner).scaled(0.5));
            String rText1 = initialTurnR > 100000 ? "" : String.format("r: %d", Math.round(initialTurnR));
            renderer.drawString3d(rText1, Color.magenta, cornerToSLMidpoint.inflate(50), 2, 2);
        }

        double timeAllotted = fRS.t - dp.t;
        Vector2 carDest = initialCarDest; //fRSCarDest
        Vector2 carToDest = carDest.minus(dp.cPf);
        Vector2 approachDirection = initialApproachDir;
        Vector2 strikeLoc = initialSL;
        double estShotAngle = bTNL.minus(strikeLoc).angleTo(approachDirection);

        // Figure out how to get there
        double desiredArcTurnR = -1; //-1 for spin line drive, >0 for arc turn
        desiredArcTurnR = initialTurnR;// + Misc.octaneHalfWidth;

        ControlsOutput cO = new ControlsOutput();
        final double maxTurnR = 50000;
        /*TODO:!defense!
        if(carToDest.magnitude() > 500 && Misc.ballTowardsNet(dp, -50, true)) { // Go back quick

        }*/
        if(desiredArcTurnR > 0) {// && desiredArcTurnR <= maxTurnR) { // Arc turn drive
            double headingOffsetMult; // sign(car2ball.angleTo(car2ownNet))
            //double aHOFAT = Misc.acuteHeadingOffsetForArcTurn(dp.cPf, carDest, desiredArcTurnR);
            double aHOFAT = initialAHOFAT;
            boolean atkAngleOK = 2.0 * aHOFAT < Math.abs(carToDest.angleTo(new Vector2(carToDest.x, 0))); // Attack angle is towards own half
            //System.out.println("[Actions.chasePushFRS] OK? " + atkAngleOK + "; 2x: " + Math.toDegrees(2.0 * aHOFAT) + ", " + Math.abs(Math.toDegrees(carToDest.angleTo(new Vector2(carToDest.x, 0)))));
            if(atkAngleOK) { // Base arc direction off of which side of ball car is facing
                headingOffsetMult = sgn(carToDest.angleTo(dp.cOf));
            } else {
                headingOffsetMult = sgn(carToDest.angleTo(Misc.ownNet(dp.team).flatten().minus(dp.cPf))); // sign(car2ball.angleTo(car2ownNet))
            }
            Vector2 absoluteDesiredHeading = carToDest.normalized().rotateBy(headingOffsetMult * aHOFAT);
            double headingErr = absoluteDesiredHeading.angleTo(dp.cOf);
            boolean headingErrSmall = Math.abs(Math.toDegrees(headingErr)) < 30;

            // Steer
            double steerMagForCurrentV = Misc.turnRadiusForSpeed(dp.cForwardVf) / desiredArcTurnR;
            double steerForDesiredR = steerMagForCurrentV * sgn(absoluteDesiredHeading.angle() - carToDest.angle()); // Assumes correct heading
            //double steerOffsetForRErr = "cutie".length(); // IT LIVES ON GOD DAMMIT
            double steerOffsetForCorrection = 1.0 * -Misc.steerForErr(headingErr);// * sgn(steerForDesiredR);
            double steer = steerForDesiredR + steerOffsetForCorrection;
            cO.withSteer(steer); //TODO: Set steering considering rErr too?

            // Dodge to get to destination?
            Vector2 c2frs = fRSPf.minus(dp.cPf);
            boolean c2frsHeadingErrOK = Math.abs(Math.toDegrees(c2frs.angleTo(dp.cOf))) < 60;
            boolean pathFollowHeadingErrOK = Math.abs(Math.toDegrees(headingErr)) < 5;
            boolean enoughTime = c2frs.magnitude() > 3000 && timeAllotted > cPFRS_minTimeToDestForFlip;
            double arcLenApprox = desiredArcTurnR > 10000 ? c2frs.magnitude() : (2.0 * Math.PI * desiredArcTurnR * (aHOFAT / Math.PI));
            boolean carApproachTOK = arcLenApprox / Math.min(dp.cVfMag + 500, 2300) > cPFRS_minTimeToDestForFlip;
            boolean carVOK = dp.cVfMag > 1200 && (dp.cVfMag < 2000 || (dp.cVfMag < 2300 && dp.car.boost < 10)) && Math.abs(Math.toDegrees(dp.cOf.angleTo(dp.cVf))) < 10;
            if(dp.car.hasWheelContact && dp.cP.z < 20 && enoughTime && c2frsHeadingErrOK && carApproachTOK && pathFollowHeadingErrOK && carVOK && desiredArcTurnR > 4000) {
                System.out.println("[Actions.chasePushFRS] Dodging to destination");
                cPFRS_lastFRS = null;
                cPFRS_lastArc = null;
                startDodge(Misc.dodgeDirection(absoluteDesiredHeading, dp.cOf), 0.08, 0.1);
                return dodge(dp);
            }

            // Dodge to hit ball? TODO: Keep working on this
            boolean opponentClose = false;
            if(dp.allCars.size() > 1) { // Not turning and at least one other car exists
                CarData otherC = dp.allCars.get(dp.playerIndex == 0 ? 1 : 0);
                Vector2 oCToBall = dp.bPf.minus(otherC.position.flatten());
                Vector2 oCVf = otherC.velocity.flatten();
                double oCSpd = oCToBall.magnitude() < 300 ? oCVf.magnitude() : oCVf.dotProduct(oCToBall.normalized());
                boolean carApproachingSoon = oCSpd != 0 && (oCToBall.magnitude() - 165) / oCSpd < (oCSpd > 2250 ? 0.7 : 0.85); // Higher val to account for possible dodge impulse
                opponentClose = carApproachingSoon;
            }
            double actualVHeadingToDest = Math.abs(Math.toDegrees(carToDest.angleTo(dp.cVf)));
            //double shotNetAngle = Math.abs(Math.toDegrees(bTNL.minus(fRSPf).angleTo(bTNL)));
            boolean ballInNetLane = Math.abs(fRSPf.x) < Misc.netHalfWidth - 150;// || (Math.abs(bTNL.x) < Misc.netHalfWidth - 150 && shotNetAngle < 45);
            boolean dodge_estShotAngleOK = Math.abs(Math.toDegrees(estShotAngle)) < 25;
            double timeLeft = Math.min(timeAllotted, carCorner.distance(strikeLoc) / (dp.cVf.isZero() ? 1 : dp.cVfMag));
            boolean dodgeEssentials = timeLeft < cPFRS_dodgeToContactT && dp.cP.z < 20 && carToDest.magnitude() < 1200;
            double netToSLDist = bTNL.distance(strikeLoc);
            boolean shouldDodgeShoot = dodgeEssentials && (opponentClose || (ballInNetLane && dodge_estShotAngleOK && netToSLDist > 2000) || (Math.abs(steer) < 0.15 && actualVHeadingToDest < 5 && desiredArcTurnR > 5000 && dodge_estShotAngleOK));
            if(shouldDodgeShoot) {
                System.out.println("[Actions.chasePushFRS] Dodging to hit ball");
                cPFRS_lastFRS = null;
                cPFRS_lastArc = null;
                //startDodge(Misc.dodgeDirection(bTNL.minus(fRSPf), dp.cOf), 0, Math.max(0, timeAllotted - (Misc.frameT * 4.0)));
                Vector2 carHitboxFront = dp.cPf.plus(dp.cOf.scaledToMagnitude(Misc.octaneLengthoffset + Misc.octaneHalfLength));
                startDodge(Misc.dodgeDirection(fRSPf.minus(carHitboxFront), dp.cOf), 0, Math.max(0, timeLeft - cPFRS_dodgeShotFlipInAdvanceT));
                return dodge(dp);
            }

            //try{Thread.sleep(100);}catch(Exception e){}
            /*System.out.println("[Actions.chasePushFRS] dATR: " + desiredArcTurnR + ", aDH: " + Math.toDegrees(absoluteDesiredHeading.angle())
                    + ", hE: " + Math.round(Math.toDegrees(headingErr)) + ", d: " + Math.round(carToDest.magnitude())
                    + ", sFDR: " + steerForDesiredR + ", cOf: " + Math.toDegrees(dp.cOf.angle())
                    + "; v heading to dest: " + Math.round(actualVHeadingToDest));*/

            // Speed (based on steer)
            double desiredV;
            double sFTR = Misc.speedForTurnRadius(desiredArcTurnR);
            double aSFATT = ArcMath.avgSpeedForArcTurnDRT(carToDest.magnitude(), desiredArcTurnR, timeAllotted);
            //desiredV = Math.min(sFTR, aSFATT);
            desiredV = aSFATT;
            if(desiredV > sFTR)// && timeAllotted > 0.5) // Avoid getting stuck right behind the ball, speed up if hitting ball soon
                desiredV = sFTR;
            //System.out.println("[Actions.chasePushFRS] desiredArcTurnR: " + Math.round(desiredArcTurnR) + "; timeAllotted: " + timeAllotted + "; cForwardVf: " + Math.round(dp.cForwardVf) + ", desiredV: " + Math.round(desiredV) + ", spd for R: " + Math.round(sFTR) + ", avg arc spd: " + Math.round(aSFATT));
            if(!headingErrSmall)
                desiredV = Math.max(desiredV - 500, 700);
            double vErr = dp.cForwardVf - desiredV; // >0 if car too fast
            double throttle;
            if(vErr > 0) // Slow down
                throttle = headingErrSmall && vErr < 300 ? 0 : -1; //OG: true
            else
                throttle = 1;

            cO.withBoost(throttle > 0 && desiredV > 1410 && vErr < -30 && dp.cVfMag < 2297);
            cO.withThrottle(throttle);
            boolean hb_velDirOK = dp.cOf.angleTo(dp.cVf) * cO.getSteer() > 0;
            cO.withHandbrake(Math.abs(Math.toDegrees(headingErr)) > 100 && dp.cForwardVf > 400 && hb_velDirOK);
            System.out.println("Making arc turn; throttle: " + throttle + ", steer: " + steer + "; headingOffsetMult > 0? " + (headingOffsetMult > 0));
        } else { // Handbrake/spin-line drive
            if(desiredArcTurnR > maxTurnR) // Just go straight
                cO = Misc.spinLineDrive(dp, carDest, timeAllotted, -1);
            else // Go to net
                cO = Misc.spinLineDrive(dp, Misc.ownNet(dp.team).flatten(), -1, -1).withBoost(false);
        }

        cPFRS_lastFRS = fRS;
        cPFRS_lastArc = arc;
        return cO;
    }

    private Vector2 cPFRS_getBallTarget(DataPacket dp, BallPredictionHelper.BPslice fRS) {
        if(dp.isKickoff && dp.allCars.size() < 2) // Kickoff and no other cars on field, don't score
            return new Vector2(4000 * -Misc.sgnNo0(dp.cP.x), 0);
        Vector2 fRSPf = fRS.p.flatten();
        double targetNetY = Misc.netY * (dp.team == 0 ? 1 : -1);

        if(Misc.sgn(fRS.v.y) * Misc.sgn(targetNetY) > 0) { // Ball going towards opponent half
            double netWallXIntercept = Misc.netWallXIntercept(targetNetY, fRSPf, fRS.v.flatten());
            boolean ballGoingIn = Math.abs(netWallXIntercept) < Misc.netHalfWidth - Misc.ballR;
            if(ballGoingIn)
                return new Vector2(netWallXIntercept, targetNetY);
        }
        Vector2 carToBall = fRSPf.minus(dp.cPf);
        Vector2 bTNLCarToBall = Misc.bestTargetNetLoc(fRS.p, fRSPf.minus(dp.cPf), dp.team).flatten();
        double bTNLCarToBallBasicStrikeAngle = Math.abs(bTNLCarToBall.minus(fRSPf).angleTo(carToBall));
        Vector2 bTNLClosest = Misc.bestTargetNetLoc(fRS.p, new Vector2(0, Misc.targetNet(dp.team).y), dp.team).flatten();
        double bTNLClosestBasicStrikeAngle = Math.abs(bTNLClosest.minus(fRSPf).angleTo(carToBall));

        Vector2 bestBTNL;
        double bestBTNLBasicStrikeAngle;
        if(Misc.ballInFrontOfNet(fRSPf)) {
            //System.out.println("[Actions.cPFRS_getBallTarget] Opted for closest net entry as net target");
            bestBTNL = bTNLClosest;
            bestBTNLBasicStrikeAngle = bTNLClosestBasicStrikeAngle;
        } else {
            bestBTNL = bTNLCarToBall;
            bestBTNLBasicStrikeAngle = bTNLCarToBallBasicStrikeAngle;
        }

        Vector2 wBL = Misc.wallShotBounceLoc(fRSPf, fRSPf.x - dp.cP.x, dp.team);
        double wBLStrikeAngle = Math.abs(wBL.minus(fRSPf).angleTo(carToBall));

        double carToBallToSideWall = Math.abs(Math.abs(Math.toDegrees(carToBall.angle())) - 90); // [0,90] 0 means carToBall parallel side wall, 90 -> perp
        if(Math.abs(fRSPf.x) < 3500 && Math.abs(wBL.y) < 3500 && carToBallToSideWall > 30 && wBLStrikeAngle < bestBTNLBasicStrikeAngle - Math.toRadians(60)) //30 Wall strike significantly better
            return wBL;
        else
            return bestBTNL;
    }

    private double cPFRS_getVelocitySLRotator(BallPredictionHelper.BPslice fRS, Vector2 bestBallTarget) {
        Vector2 bVf = fRS.v.flatten();
        Vector2 ballToTarget = bestBallTarget.minus(fRS.p.flatten());
        double vMag = bVf.magnitude();
        //double vParallelComponent = Math.abs(bVf.dotProduct(ballToTarget.normalized())); // >0 forward
        //double vPerpendicularComponent = Math.abs(bVf.dotProduct(ballToTarget.normalized().rotateBy(DataPacket.relativeCoordsXRotator)));
        //Vector2 bVfRelToBallToTarget = new Vector2(vPerpendicularComponent, vParallelComponent);
        double vAngleErr = ballToTarget.angleTo(bVf);
        //double vPerpendicularFraction = Math.abs(Math.sin(vAngleErr));
        boolean ballGoingAwayFromNet = Math.abs(vAngleErr) > Math.PI / 2.0;

        final double rotMaxBallV = 2000;//2000?, 3000
        // Mirrors around 90deg; vAngleErr==90deg -> 45deg, vAngleErr==60deg -> 30deg, vAngleErr==120deg -> 15deg, etc.

        // NEW TODO: Keep working
        double multiplier = 0.2 + (0.65 * Math.min(vMag, rotMaxBallV) / rotMaxBallV);
        double rotator;
        if(ballGoingAwayFromNet) {
            rotator = -sgn(vAngleErr) * ((Math.PI / 2.0) - Math.abs(Math.abs(vAngleErr) - (Math.PI / 2.0))) / 2.0;// / 2.0
            rotator *= Misc.clamp(multiplier, 0, 1);
        } else {
            rotator = vAngleErr / 2.0;
            rotator = -sgn(rotator) * Misc.clamp(Math.abs(rotator) * multiplier, 0, Math.PI / 2.0);
        }

        /*// OG
        double rotator;
        if(!ballGoingAwayFromNet) {
            double multiplier = 0.3 + 1.5 * (Math.min(vMag, rotMaxBallV) / rotMaxBallV);
            rotator = -sgn(vAngleErr) * Misc.clamp((Math.PI / 2.0) * multiplier, 0, Math.PI / 4.0);
        } else {
            //System.out.println("[Actions.cPFRS_getVelocitySLRotator] Ball coming at car");
            rotator = -sgn(vAngleErr) * ((Math.PI / 2.0) - Math.abs(Math.abs(vAngleErr) - (Math.PI / 2.0))) / 2.0;
        }*/

        //System.out.println("[Actions.cPFRS_getVelocitySLRotator] SLRotator: " + rotator);
        return rotator;
    }


    /*
        Dribble (catch, carry, flick)
     */
    // Flick code
    private double flickStartT, flickFinishT, flickWheelContactT;
    private int flickFlipFramesWaited; // Between initial jump and flip
    private boolean flickJumped; // Initial jump started
    public void startFlick() {
        flickLock = true;
        flickStartT = -1;
        flickFlipFramesWaited = 0;
        flickJumped = false;
        flickFinishT = -1;
        flickWheelContactT = -1;
        System.out.println("STARTING FLICK");
    }
    public ControlsOutput flick(DataPacket dp) {
        if(flickFinishT != -1) {
            if(dp.car.hasWheelContact && flickWheelContactT == -1)
                flickWheelContactT = dp.t;
            if(flickFinishT + Misc.postFlipMaxWaitTime < dp.t || (flickWheelContactT != -1 && dp.t - flickWheelContactT > Misc.postFlipWheelContactTimeout)) { // Wait for flip to complete
                flickLock = false;
            }
            return new ControlsOutput();
        }

        if (flickStartT < 0)
            flickStartT = dp.t;
        double elapsed = dp.t - flickStartT;
        ControlsOutput cO = new ControlsOutput();

        final double flickJumpHoldT = 0.2;
        if(elapsed > 1.25 + flickJumpHoldT || dp.car.doubleJumped) {
            System.out.print("FLICKING TIMED OUT!");
            flickLock = false;
        } else if(elapsed < flickJumpHoldT || !flickJumped) { // Initial jump (min 1 frame)
            System.out.print("FLICKING (initial jump);");
            cO.withJump();
            cO.withYaw(-1);
            //cO.withRoll(0.2);
            cO.withBoost(elapsed > 0.05);
            flickJumped = true;
        } else if(flickJumped && flickFlipFramesWaited < Misc.minWaitFramesBeforeFlip) { // One frame with no jump input
            System.out.print("FLICKING (wait frame);");
            flickFlipFramesWaited++;
        } else { // Initial jump done (and waited one frame), do actual flick
            Vector3 roofO = dp.car.orientation.roofVector;
            Vector3 carToBallN = dp.bP.minus(dp.cP).normalized();
            //boolean roofZOK = roofO.z < carToBallN.z || Math.abs(roofO.z - carToBallN.z) < 0.2;
            //double a = Math.toDegrees(Math.abs(Math.abs(roofO.flatten().angleTo(carToBallN.flatten())) - 90));
            //boolean shouldDodge = roofZOK && dp.cO.z > -0.3;
            //boolean shouldDodge = roofO.z < 0.8 && dp.cO.z > -0.2;
            boolean shouldDodge = Math.abs(Math.toDegrees(dp.cOf.angleTo(dp.cVf))) > 160;
            if(shouldDodge) {
                Vector2 flickDir = new Vector2(0,-1); // 0,-1 is backflip
                System.out.print("FLICKING (flip!);");
                cO.withJump();
                cO.withPitch(-Math.sin(flickDir.angle()));
                cO.withYaw(Math.cos(flickDir.angle()));
                flickFinishT = dp.t;
            } else {
                //cO.withPitch(-0.1); // >0 Nose up
                cO.withYaw(-1); // >0 Right
                //cO.withRoll(0.2); // >0 Right
                //cO.withBoost(elapsed - flickJumpHoldT > 0.0);
                System.out.print("FLICKING (waiting);");
            }
        }

        System.out.println(" " + Math.round(elapsed * 1000.0) + "ms elapsed");
        return cO;
    }

    // Constants
    private static final double minTurnTargetXOffRelMag = 20; // for turn, 20
    private static final double maxTurnTargetXOffRelMag = 31 + Misc.octaneHalfWidth; //33? 31 halfWidth is 42
    private static final double maxCatchTargetXOffRelMag = 30; //30
    private static final double minUnstableTargetYOffRel = -35;
    private static final double minTargetYOffRel = -(5 + Misc.octaneHalfLength); //halfLen is 59
    private static final double idealYOffRel = -40; //0 -55 //-Misc.octaneLengthoffset
    private static final double idealSharpTurnYOffRel = -10; //0, -25? //-Misc.octaneLengthoffset
    private static final double maxTargetYOffRel = 30; //0, 5
    private static final double idealBallSpeed = 1700; //950? 900 for idealYOffRel=0
    private static final double idealTurnBallSpeed = 850; //800? 850
    private static final double idealBallSpeedTolerance = 50;//5

    private static final double approxFlickRange = 3200;
    private static final double targetFlickYOffRel = -65; //65-66 max for throttle=1 for entire 0.2s initial jump, ~45 for -1, ~57 for 0
    //(yOffRel, throttle): observed flipYOffRel; 65, 1: 75.79(bad), 73.65(ok), 76.94(bad), 75.12(ok/bad), 74.38(ok), 75.45(ok); 66, 1: 78.62, 76.29, 76.94; 58, 0: 75.14, 74.56; ideal: 74? (at flip time)
    //more 65, 1 (start,flip): 65.57,76.13; 65.21, 77.30; 65.60,97.67(bouncy); 65.06,76.08(weak); 65.25,76.49; 65.78,74.81; 65.82,74.51(good);
    private static final double flickYOffRelTolerance = 1;
    private static final double targetFlickBallSpeed = 1100; //1000? 1100

    private double lastCRelY, lastCRelX;
    private boolean preparingFlick = false;
    private ControlsOutput carry(DataPacket dp) {
        double dt = dp.dt;
        double heightAboveCarFront = dp.bP.z - (Misc.octaneFrontTopHeight + Misc.ballR + (dp.car.position.z - Misc.octaneJointHeight));
        //Vector2 bTNL = Misc.bestTargetNetLoc(dp.bP, dp.bVf, 100, dp.team).flatten();
        Vector2 bTNL = Misc.targetNet(dp.team).flatten();
        double ballAngleToNetDeg = Math.toDegrees(dp.bVf.angleTo(bTNL.minus(dp.bPf)));
        ballAngleToNetDeg = (Math.abs(ballAngleToNetDeg) < 0.1 ? 0 : ballAngleToNetDeg);
        double carToBallVHeadingErr = dp.cOf.angleTo(dp.bVf);
        boolean stableCarry = heightAboveCarFront < 10 && Math.abs(dp.bV.z) < 80;// && heightAboveCarFront > 0;
        boolean turnSharply = Math.abs(ballAngleToNetDeg) > 5 && dp.bPf.distance(bTNL) > 150; //5 Put ball on side of car
        double currentMinTargetYOffRel = dp.car.boost > 10 ? minTargetYOffRel : -35;
        double currentIdealYOffRel = dp.car.boost > 10 ? idealYOffRel : -25;

        // Pop?
        if(!turnSharply && dp.allCars.size() > 1) { // Not turning and at least one other car exists
            CarData otherC = dp.allCars.get(dp.playerIndex == 0 ? 1 : 0);
            Vector2 oCToBall = dp.bPf.minus(otherC.position.flatten());
            boolean carFacingBall = Math.abs(Math.toDegrees(oCToBall.angleTo(otherC.orientation.noseVector.flatten()))) < 30;
            double oCSpd = otherC.velocity.flatten().magnitude();
            boolean carApproachingSoon = oCSpd != 0 && (oCToBall.magnitude() - 165) / oCSpd < (oCSpd > 2250 ? 0.7 : 0.85);
            boolean opponentFastOrNotBehind = oCSpd > dp.cVfMag || Math.abs(Math.toDegrees(oCToBall.angleTo(dp.cOf))) > 45;
            if(carFacingBall && carApproachingSoon && opponentFastOrNotBehind) {
                startDodge(new Vector2(0,0), 0, 0, true);
                return dodge(dp);
            }
        }

        // Flick?
        double spdDiff = dp.cVfMag - dp.bVfMag;
        double headingErr = Math.abs(Math.toDegrees(carToBallVHeadingErr));
        if(stableCarry && !turnSharply && headingErr < 3 && spdDiff > -20 && spdDiff < 30 && dp.bVfMag > 700 && dp.cRelY > -60 && dp.cRelY < -20)// && spdDiff > 50
            startFlick();

        // Desired speed offset control
        double targetBallV = preparingFlick ? targetFlickBallSpeed : (turnSharply ? idealTurnBallSpeed : idealBallSpeed);
        double ballVErr = dp.bVfMag - targetBallV; // Positive if ball too fast
        if(Math.abs(ballVErr) < idealBallSpeedTolerance)
            ballVErr = 0;
        double targetYOffRel;
        if(preparingFlick && ballVErr > -150) { //-200
            targetYOffRel = targetFlickYOffRel;
        } else {
            if(ballVErr == 0 || turnSharply)
                targetYOffRel = false && turnSharply ? idealSharpTurnYOffRel : currentIdealYOffRel;// + (ballVErr / 4.0);
            else
                targetYOffRel = ballVErr / (ballVErr > 0 ? 12.0 : 5.0);
            targetYOffRel = Misc.clamp(targetYOffRel, stableCarry ? currentMinTargetYOffRel : minUnstableTargetYOffRel,  maxTargetYOffRel);
        }
        System.out.println("[Actions.carry] cRelY: " + Math.round(dp.cRelY) + ", target: " + Math.round(targetYOffRel) + ", ballVErr: " + Math.round(ballVErr));
        double yOffRelErr = dp.cRelY - targetYOffRel; // Positive if ball too far backward
        double lastYOffRelErr = lastCRelY - targetYOffRel;
        double P = -(5.7 + Math.max(0, Math.pow(dp.cRelY, 5) / 100000000.0)); // P, 6.0? TODO: Make logarithmic?
        double desiredVOffset = yOffRelErr * P;
        if(dt < Misc.frameT * 1.5 && dt > 0) {// && !couldFlick) { // Don't use if trying to flick TODO: Change?
            desiredVOffset -= 0.5 * (yOffRelErr - lastYOffRelErr) / dt; // D, 0.5?
            //System.out.println("[Actions.carry] Using derivative for desiredVOffset offset: " + (-0.5 * ((yOffRelErr - lastYOffRelErr) / dt)));
        }

        // Speed control
        double desiredCarV = dp.bVfMag + desiredVOffset;
        double carV = dp.cForwardVf;
        if(carV > 0 && !dp.bVf.isZero())
            carV = dp.car.velocity.flatten().dotProduct(dp.bVf.normalized());
        double carVErr = carV - desiredCarV; // Positive if car too fast
        double throttleOutput;
        boolean boostOutput = false;
        if(carVErr > 450 && (dp.car.boost > 6 || yOffRelErr < 0)) // OG just carVErr > 250
            throttleOutput = -1;
        else if(carVErr > 50) // was 150
            throttleOutput = 0;
        else if(carVErr > 0)
            throttleOutput = carV > 1410 ? 0.02 : 0;
        else {
            throttleOutput = 1;
            double dYOffRel = (dp.cRelY - lastCRelY) / dt;
            boolean ballStuckForward = !preparingFlick && dp.cRelY < currentMinTargetYOffRel && dYOffRel < 20; // -60 == ball moving backwards 60 units per second
            if(ballStuckForward)
                System.out.println("[Actions.carry] Ball stuck foward");
            boostOutput = ballStuckForward || carVErr < (desiredCarV < 1410 ? -350 : -100); //130
        }


        // Desired xOffRel control
        //TODO: Hardcode targetXOffRel to a constant when preparingFlick?
        //TODO: Turn less sharp when enemies are close to allow for flicking
        double targetXOffRel = 0;
        final double xOffRelModMult = -2.0; // OG 2.0 (-> 21deg), 0.7 -> 60deg, 1.2 -> 35deg
        targetXOffRel = Misc.clamp(ballAngleToNetDeg * xOffRelModMult, maxTurnTargetXOffRelMag);
        if(turnSharply) { // OG 5, Put the ball off the side of the car to turn faster
            targetXOffRel *= Math.max(0, maxTurnTargetXOffRelMag - minTurnTargetXOffRelMag) / maxTurnTargetXOffRelMag; // Scale to the dist between min and max
            targetXOffRel += Misc.sgn(targetXOffRel) * minTurnTargetXOffRelMag; // Add min
            targetXOffRel = Misc.clamp(targetXOffRel, maxTurnTargetXOffRelMag); // Should be unnecessary
        } else if(false) {
            Vector2 oppositeGoalCorner = new Vector2((Misc.netHalfWidth - 100) * (Math.abs(bTNL.x) < 200 ? 1.0 : -1.0) * sgn(bTNL.x), bTNL.y);//Misc.targetNet(dp.team).minus(dp.bP).flatten();
            double ballAngleToNetCenter = dp.bVf.angleTo(oppositeGoalCorner);
            targetXOffRel = 20.0 * -sgn(ballAngleToNetCenter);
        }
        if((!stableCarry) || dp.cRelX * targetXOffRel < 0 ) { // Get under the ball when it is falling or on wrong side of car
            targetXOffRel = Misc.clamp(targetXOffRel, maxCatchTargetXOffRelMag);
            System.out.println("[Actions.carry] Clamping targetXOffRel for catch");
        }

        // Steer control
        final double xP = 1.0 / 30.0; //30 TODO: exponential? (250)
        final double mhP = 0.4; //0.8
        final double xD = 1.0 / 120.0; //120
        double xErr = dp.cRelX - targetXOffRel; // Positive if ball too far right
        double steerOutput = (xP * xErr) + (mhP * carToBallVHeadingErr);
        //System.out.println("[Actions.carry] Steering: xP component: " + (xFrac * 100.0) + "% of " + (xP * xErr) + " = " + (xFrac * (xP * xErr)) + " (xErr = " + xErr + ")");
        //System.out.println("[Actions.carry] Steering: mhP component: " + (mhFrac * 100.0) + "% of " + (mhP * carToBallVHeadingErr) + " = " + (mhFrac * (mhP * carToBallVHeadingErr)) + " (carToBallVHeadingErr = " + carToBallVHeadingErr + ")");
        if(dt < Misc.frameT * 1.5 && dt > 0) { // Use derivative for x
            double lastXErr = lastCRelX - targetXOffRel; // 3 is a lot
            double dXErr = (xErr - lastXErr) / dt; // Ball going more left if positive
            steerOutput += xD * dXErr;
            //System.out.println("[Actions.carry] Steering: xD component: " + (xFrac * 100.0) + "% of " + (xD * dXErr) + " = " + (xFrac * (xD * dXErr)) + " (dXErr = " + dXErr + ")");
        }

        lastCRelY = dp.cRelY;
        lastCRelX = dp.cRelX;
        //System.out.println("[Actions.carry] xOffRel: " + Math.round(dp.cRelX) + ", yOffRel: " + Math.round(dp.cRelY) + "; targetXOffRel: " + (-targetXOffRel) + ", targetYOffRel: " + targetYOffRel + ", yOffRelErr: " + yOffRelErr + "; heightAboveCarFront: " + heightAboveCarFront + "; dt: " + dt);
        //System.out.println("[Actions.carry] desiredVOffset: " + desiredVOffset + ", desiredCarV: " + Math.round(desiredCarV) + ", carVErr: " + carVErr + "; ballVfMag: " + dp.bVfMag + ", carVfMag: " + dp.cVfMag + "; carV: " + carV + "; ballAngleToNetDeg: " + ballAngleToNetDeg);
        return new ControlsOutput()
                .withThrottle(throttleOutput)
                .withSteer(steerOutput)
                .withBoost(boostOutput);
    }

    private BallPredictionHelper.BPslice lastFCS;
    public boolean dribbling = false, catching = false;
    // Handler and catch code
    public ControlsOutput dribble(DataPacket dp, BallPredictionHelper.BPslice fCS) {
        if(dribbling) { // Dribble
            catching = false;
            return carry(dp);
        } else {
            // Catch
            if (!catching) { // First run
                System.out.println("[Actions.dribble] Starting catch");
                catching = true;
            }
            // Catching
            BallPredictionHelper.BPslice targetFCS = lastFCS;
            if (fCS == null && lastFCS.t - dp.t > 0.5) {
                System.out.println("[Actions.dribble] Not dribbling and can't catch, no catchable slice!");
                catching = false;
                return new ControlsOutput();
            } else {
                boolean lastFCSStale = lastFCS == null || dp.newBallTouch || BallPredictionHelper.posErr(dp, lastFCS) > 10;
                if (lastFCSStale || lastFCS.t - dp.t > 1.2 || fCS.t > lastFCS.t + 0.1) { // Can't reach lastFCS, abandon it
                    targetFCS = fCS;
                }
            }

            /*
            double timeAllotted = -1;//
            ControlsOutput cO = Misc.spinLineDrive(dp, targetFCS.p.flatten(), timeAllotted, 0);
            */
            double timeAllotted = targetFCS.t - dp.t;
            double endV = targetFCS.v.flatten().dotProduct(dp.cOf); //TODO: When far, carToTargetFRS instead of dp.cOf?
            Vector2 carDest = targetFCS.p.flatten(); //TODO: 9:52 https://www.youtube.com/watch?v=fa_HCBhoheY
            ControlsOutput cO = Misc.spinLineDrive(dp, carDest, timeAllotted, endV);

            lastFCS = targetFCS;
            return cO;
        }
    }

    private BallPredictionHelper.BPslice firstCatchableSlice(DataPacket dp, double tMax) {
        BallPredictionHelper.BPslice[] bP = dp.bPred();
        float now = dp.t;
        for(int i = 1; i < bP.length - 1; i++) {
            if(bP[i].t > tMax)
                break;
            // Ball going up, ball below roof height, or ball will still be above roof height next slice: skip
            if(bP[i].v.z > 0 || bP[i].p.z < Misc.octaneFrontTopHeight + Misc.ballR || bP[i + 1].p.z > Misc.octaneFrontTopHeight + Misc.ballR)
                continue;

            Vector2 positionToBall = bP[i].p.flatten().minus(dp.cPf);
            double d = positionToBall.magnitude();

            //TODO: Change eventually for dodges
            double avgRequiredStraightLineSpd = d / (bP[i].t - now);
            if(avgRequiredStraightLineSpd > 2300)
                continue;

            double bVfMag = bP[i].v.flatten().magnitude();
            double bVZ = bP[i].v.z;
            boolean falling = (-bVZ / bVfMag) > 0.9;
            if(!falling || bVfMag > 1200)
                continue;

            double vi = dp.cVf.dotProduct(positionToBall.normalized());
            //long t2 = System.nanoTime();
            double approachT = Misc.lineDriveDuration(vi, d, dp.car.boost, bP[i].v.flatten().magnitude());
            //System.out.println("firstCatchableSlice t2 took " + ((System.nanoTime() - t2) / 1000000.0) + "ms");
            if(now + approachT < bP[i].t) { // Reachable or last slice (6 seconds out)
                //System.out.println(i + ": d=" + d + ", vi=" + vi + ", t=" + approachT);
                //System.out.println("firstCatchableSlice to " + i + " took " + ((System.nanoTime() - t0) / 1000000.0) + "ms");
                return bP[i];
            }
        }
        return null;
    }
}
