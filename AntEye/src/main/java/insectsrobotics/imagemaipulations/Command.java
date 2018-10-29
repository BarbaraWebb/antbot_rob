package insectsrobotics.imagemaipulations;

import insectsrobotics.imagemaipulations.Receiver_and_Broadcaster.Broadcast;
import static java.lang.Thread.sleep;

public class Command {
    //
    // Singleton construction
    //
    private static Command instance = new Command();
    private Command(){}


    //
    // Private Members
    //

    // Required to communicate with the hardware. Will be set in MainActivity
    // and passed in later.
    private static Broadcast broadcast;

    //
    // Public Methods
    //

    // Retrieve the static instance of this class
    public static Command getInstance(){ return instance; }

    //
    // Set the serial broadcast object to be used for commands
    //
    public static void setBroadcast(Broadcast broadcastPool)
    {
        broadcast = broadcastPool;
    }

    //
    // Drive forward
    //
    public static void moveForward(double dist)
    {
        try
        {
            sleep(100);
        }
        catch(InterruptedException e)
        {
            e.printStackTrace();
        }

        broadcast.broadcastDistance(dist);
    }

    //
    // Drive indefinitely with set speeds
    //
    public static void go(double[] speeds)
    {
        try
        {
            sleep(100);
        }
        catch(InterruptedException e)
        {
            e.printStackTrace();
        }

        broadcast.broadcastMove(speeds);
    }

    //
    // Turn by an angle
    //
    public static void turnAround(double theta) throws InterruptedException {
        try
        {
            sleep(100);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        if(theta > 180){
            theta = theta - 360;
        }else if(theta < -180){
            theta = theta + 360;
        }

        broadcast.broadcastAngle(theta);
        sleep(3000);
    }


}
