package Journey;

import rlbot.Bot;
import rlbot.ControllerState;
import rlbot.cppinterop.RLBotDll;
import rlbot.cppinterop.RLBotInterfaceException;
import rlbot.flat.BallPrediction;
import rlbot.flat.GameTickPacket;
import rlbot.flat.QuickChatSelection;
import rlbot.manager.BotLoopRenderer;
import rlbot.render.Renderer;
import Journey.boost.BoostManager;
import Journey.input.CarData;
import Journey.input.DataPacket;
import Journey.output.ControlsOutput;
import Journey.prediction.BallPredictionHelper;
import Journey.vector.Vector2;

import java.awt.Color;

public class Journey implements Bot {

    private final int playerIndex;

    public Journey(int playerIndex) {
        this.playerIndex = playerIndex;
    }

    /**
     * This is where we keep the actual bot logic. This function shows how to chase the ball.
     * Modify it to make your bot smarter!
     */
    private ControlsOutput processInput(DataPacket dp) {

        Vector2 ballPosition = dp.bPf;
        Vector2 carPosition = dp.cPf;
        Vector2 carDirection = dp.cOf;

        // Subtract the two positions to get a vector pointing from the car to the ball.
        Vector2 carToBall = ballPosition.minus(carPosition);

        // How far does the car need to rotate before it's pointing exactly at the ball?
        double steerCorrectionRadians = carDirection.angleTo(carToBall);

        boolean goLeft = steerCorrectionRadians > 0;

        // This is optional!
        doRendering(dp, dp.car, goLeft);

        // This is also optional!
        if (dp.ball.position.z > 300) {
            RLBotDll.sendQuickChat(playerIndex, false, QuickChatSelection.Compliments_NiceOne);
        }

        return new ControlsOutput()
                .withSteer(goLeft ? -1 : 1)
                .withThrottle(1);
    }

    private void doRendering(DataPacket dp, CarData myCar, boolean goLeft) {
        // Here's an example of rendering debug data on the screen.
        Renderer renderer = BotLoopRenderer.forBotLoop(this);

        // Draw a line from the car to the ball
        renderer.drawLine3d(Color.LIGHT_GRAY, myCar.position, dp.ball.position);

        // Draw a line that points out from the nose of the car.
        renderer.drawLine3d(goLeft ? Color.BLUE : Color.RED,
                myCar.position.plus(myCar.orientation.noseVector.scaled(150)),
                myCar.position.plus(myCar.orientation.noseVector.scaled(300)));

        renderer.drawString3d(goLeft ? "left" : "right", Color.WHITE, myCar.position, 2, 2);

        // Draw 3 seconds of ball prediction
        BallPredictionHelper.drawTillMoment(dp.t + 3, Color.CYAN, renderer);
    }


    @Override
    public int getIndex() {
        return this.playerIndex;
    }

    /**
     * This is the most important function. It will automatically get called by the framework with fresh data
     * every frame. Respond with appropriate controls!
     */
    @Override
    public ControllerState processInput(GameTickPacket packet) {

        if (packet.playersLength() <= playerIndex || packet.ball() == null || !packet.gameInfo().isRoundActive()) {
            // Just return immediately if something looks wrong with the data. This helps us avoid stack traces.
            return new ControlsOutput();
        }

        // Update the boost manager and tile manager with the latest data
        BoostManager.loadGameTickPacket(packet);

        // Translate the raw packet data (which is in an unpleasant format) into our custom DataPacket class.
        // The DataPacket might not include everything from GameTickPacket, so improve it if you need to!
        DataPacket dataPacket = new DataPacket(packet, playerIndex);

        // Do the actual logic using our dataPacket.
        ControlsOutput controlsOutput = processInput(dataPacket);

        return controlsOutput;
    }

    public void retire() {
        System.out.println("Retiring sample bot " + playerIndex);
    }
}
