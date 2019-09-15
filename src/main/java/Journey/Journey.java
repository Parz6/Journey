package Journey;

import Journey.boost.BoostPad;
import Journey.core.Actions;
import Journey.output.CarController;
import Journey.util.Misc;
import rlbot.Bot;
import rlbot.ControllerState;
import rlbot.flat.GameTickPacket;
import rlbot.manager.BotLoopRenderer;
import rlbot.render.Renderer;
import Journey.boost.BoostManager;
import Journey.input.DataPacket;
import Journey.output.ControlsOutput;

import java.awt.Color;
import java.awt.Point;

public class Journey implements Bot {
    private static final boolean ALLOW_CONTROLLER_OVERRIDE = false;

    private final int playerIndex;
    private DataPacket lastDP;
    private CarController carController;
    public Testing testing;
    public Actions actions;
    public double lastKickoffTime;


    //TODO List:
    //TODO: pushballtonet code, hookshots, powershots (use next bounce slice), defensive intercept code if Misc.ballTowardsNet(dp, true)
    //TODO: get boost near arc or in front of car (both types, less constraints for full)
    //TODO: Stop using fRS in Testing.chasePushFRS(), loop through ballPrediction within chasePushFRS() itself;
    //          find slice with arc where avgV needed is just under Misc.speedForTurnRadius(initialTurnR) (and next slice is over)
    //TODO: Other TODOs (gotem), such as ball prediction memory usage

    public Journey(int playerIndex) {
        this.playerIndex = playerIndex;
        this.carController = new CarController(ALLOW_CONTROLLER_OVERRIDE);
        actions = new Actions(this);
        testing = new Testing(this);
        //Testing.scenario2(playerIndex);
    }

    private ControlsOutput processInput(DataPacket dp) {
        Renderer renderer = BotLoopRenderer.forBotLoop(this);
        if(false && renderer != null) {
            long tRenderStart = System.nanoTime();
            doRendering(dp, renderer);
            System.out.println("[Journey.processInput] doRendering took " + ((System.nanoTime() - tRenderStart) / 1000.0) + "us");
        }

        ControlsOutput cO;
        BoostPad tFBP = BoostManager.getTargetFullBoostPad(dp);
        if(dp.isKickoff && dp.allCars.size() > 1) {                             // Kickoff
            cO = Actions.kickoff(dp);
        } else if(dp.t - lastKickoffTime < Misc.postFlipMaxWaitTime) {          // Wait for kickoff dodge timeout
            cO = new ControlsOutput();
        } else if(actions.actionLockExists()) {                                 // Handle locked action
            cO = actions.handleLocks(dp);
        } else if(!dp.car.hasWheelContact) {                                    // Rotate to land on wheels
            cO = actions.airRecovery(dp, renderer);
        } else if(Math.abs(dp.cP.y) > Misc.netY + 30) {                         // Escape net
            cO = Actions.escapeNet(dp, renderer);
        } else if(Actions.shouldEscapeWall(dp)) {                               // Escape wall
            cO = Actions.escapeWall(dp, renderer);
        } else if(Actions.shouldPushBallIn(dp)) {                               // Push ball in
            cO = actions.pushBallIn(dp, renderer);
        } else if(tFBP != null && Actions.shouldGetBoost(dp, tFBP)) {           // Get Boost TODO: Finish
            cO = Actions.getBoostPad(dp, renderer, tFBP);
        } else {                                                                // Normal behavior
            cO = actions.runBotCore(dp, renderer);
        }
        carController.setControls(cO);
        return carController.getControls();
    }

    private void doRendering(DataPacket dp, Renderer renderer) {
        if(carController.isHumanControlled())
            renderer.drawString2d("Human", Color.magenta, new Point(3, 988), 6, 6);

        //BallPredictionHelper.drawUntilMoment(dp, dp.t + 3, Color.CYAN, renderer); // Draw 3 seconds of ball prediction
        //BallPredictionHelper.draw(dp, renderer);
    }


    @Override
    public int getIndex() {
        return this.playerIndex;
    }

    public int framesElapsed = 0, lastTouchFrameNum = -1, maxFrameTNumber = -1;
    public double maxFrameT = 0;
    @Override
    public ControllerState processInput(GameTickPacket packet) { // Per-frame entry point, called by framework
        // Just return immediately if something looks wrong with the data. This helps us avoid stack traces.
        if(packet.playersLength() <= playerIndex || packet.ball() == null || !packet.gameInfo().isRoundActive())
            return new ControlsOutput().withThrottle(1).withBoost(); // To start kicking off with no delay

        long pIStart = System.nanoTime();
        BoostManager.loadGameTickPacket(packet); // Update the boost manager and tile manager with the latest data
        DataPacket dp = new DataPacket(packet, playerIndex, lastDP); // Wrap the raw packet data
        if(dp.isKickoff)
            lastKickoffTime = dp.t;
        if(dp.newBallTouch)
            lastTouchFrameNum = framesElapsed;
        ControlsOutput controlsOutput = processInput(dp); // Request controls and provide fresh game data

        double elapsedT = (((double) (System.nanoTime() - pIStart)) / 1000000.0); // In ms
        if(elapsedT > maxFrameT && framesElapsed > 0) {
            maxFrameT = elapsedT;
            maxFrameTNumber = framesElapsed;
        }
        String elapsedText = String.format("%.3fms (max: %.3fms on #%d)", elapsedT, maxFrameT, maxFrameTNumber);
        System.out.println("======  Frame " + framesElapsed + " took " + elapsedText + "; last ball touch on #" + lastTouchFrameNum + ")  ======");

        lastDP = dp;
        framesElapsed++;
        return controlsOutput;
    }

    public void retire() {
        System.out.println("Retiring " + this.getClass().getSimpleName() + " (index " + playerIndex + ")");
    }
}
