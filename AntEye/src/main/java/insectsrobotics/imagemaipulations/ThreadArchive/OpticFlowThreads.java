package insectsrobotics.imagemaipulations.ThreadArchive;

import insectsrobotics.imagemaipulations.MainActivity;
import insectsrobotics.imagemaipulations.Util;
import insectsrobotics.imagemaipulations.Command;
import insectsrobotics.imagemaipulations.StatFileUtils;

import android.util.Log;
import android.os.SystemClock;

import static java.lang.Thread.sleep;
public class OpticFlowThreads {
    private MainActivity main;
    //
    // Constructor. For legecay reasons we need access to MainActivity for various global variables
    //
    public OpticFlowThreads(MainActivity main){
        this.main = main;
    }

    public Runnable opticalFlowAvoidance = new Runnable() {
        @Override
        public void run() { //Should really be worked in with existing PI stuff
            //This version of the thread is for testing the flow filtering collision avoidance
            try {
                sleep(3000);
            } catch (Exception e){
                e.printStackTrace();
            }

            Util.log(main.log_file, "CA run started");
            boolean avoid = false; //Is the robot currently avoiding an obstacle?
            boolean stop = false; //Halt flag
            boolean initialise = true; //Loop initialisation flag
            int start_time = (int) SystemClock.elapsedRealtime(); //Start time for the thread
            int current_time = (int) SystemClock.elapsedRealtime() - start_time; //Current thread time
            int t_move_start = 0; //Time that a saccade started

            //Accumulation time intervals
            int t_interval_start = current_time; //Start of the time interval
            int t_delta = (int) SystemClock.elapsedRealtime() - current_time; //Time interval

            //Left and right motor speeds
            int lft_speed = 15;
            int rgt_speed = 14;

            //Accumulators for left and right flow
            int dual_accumulator = 0; //Add both values and see if there's significant bias
            int left_accumulator = 0; //Add only left values
            int right_accumulator = 0; //Add only right values
            double accumulation_threshold = 0.04; //Threshold for a value to be accumulated (ignore all others) (def = 5000)
            double reaction_threshold = 0.8; //Value to be met for a reaction to be triggered. (need at most four readings) (def = 10000)
            int turn = 20;

            boolean reaction_taken = false;
            int reaction_time;

            int loop_count = 0;


            //While
            while ( !stop && (current_time <= 30000) ){
                try { sleep(600); } catch ( Exception e ){ e.printStackTrace(); }

                //Print out debug information, catch exceptions
                main.ca_flow_diff = main.leftCAFlow - main.rightCAFlow; //If positive, left detection, else right detection

                //Flow accumulators
                if ( main.ca_flow_diff >= accumulation_threshold ){ left_accumulator = left_accumulator + (int) main.ca_flow_diff; }
                else if ( main.ca_flow_diff <= -accumulation_threshold ){ right_accumulator = right_accumulator + Math.abs((int) main.ca_flow_diff); }

                if ( Math.abs(main.ca_flow_diff) >= accumulation_threshold ){ dual_accumulator = dual_accumulator + (int) main.ca_flow_diff; }

                //If accumulation time limit reached, reset timer and accumulators
                if ( loop_count >= 2/*t_delta >= 2000 */){
                    left_accumulator = 0;
                    right_accumulator = 0;
                    dual_accumulator = 0;
                    loop_count = 0; //Reset loop counter
                    t_interval_start = (int) SystemClock.elapsedRealtime();
                }

                try {
                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run(){
                            main.debugTextView.setText(String.format(
                                    "Speed: %f \n" +
                                            "Left CA flow: %f \n" +
                                            "Right CA flow: %f \n" +
                                            "Flow Difference: %f \n"
                                    ,
                                    main.speed,
                                    main.leftCAFlow,
                                    main.rightCAFlow,
                                    main.ca_flow_diff
                            ));
                        }
                    });
                } catch ( Exception e ){
                    e.printStackTrace();
                }
                //End printing

                //Initialise if 1st iteration (re-used to reset speeds mid-run)
                if ( initialise ){
                    initialise = false;
                    Command.go( new double[] { lft_speed, rgt_speed } );
                    try{ sleep(1000); } catch ( Exception e ){ e.printStackTrace(); } //Command delay
                    t_interval_start = (int) SystemClock.elapsedRealtime(); //Start the time interval for accumulation
                }

                //Figure out which way to turn
                boolean left = false;
                boolean right = false;

                //Dual decision (contiguous results)
                //if ( dual_accumulator >= reaction_threshold ){ right = true; }
                //else if ( dual_accumulator <= -reaction_threshold ){ left = true; }



                //Uncomment to use the individual method for accumulation.
                //Individual decision
                if ( left_accumulator >= reaction_threshold ){ right = true; }
                else if ( right_accumulator >= reaction_threshold ){ left = true; }
                //Left accumulator keeps track of left flow and so should trigger a right turn
                //Right accumulator keeps track of right flow and so should trigger a left turn


                if ( left && (!avoid) ){ //Left turn required
                    Log.i("CA:", "Left turn triggered");
                    if ( !reaction_taken ){
                        reaction_taken = true;
                        reaction_time = (int) SystemClock.elapsedRealtime() - t_interval_start;
                        //Reaction time in milliseconds. Print to stats file
                        StatFileUtils.write("OF", "RCTIME", "Reaction time: " + reaction_time + "; for speeds {" + lft_speed + ", " + rgt_speed + "}");
                    }

                    Command.go( new double[]{0, 0});
                    try { sleep(1000); } catch( Exception e ){ e.printStackTrace(); }
                    try {
                        Command.turnAround(turn);
                    }catch(Exception e){ e.printStackTrace(); }
                    /*
                    lft_speed = 10; //Left turn
                    rgt_speed = 100; //Left turn*/
                    initialise = true; //Re-call the go command to trigger the speed change
                    avoid = true; //Flag so we know to ignore this code snippet when avoiding
                    dual_accumulator = 0;
                    left_accumulator = 0;
                    right_accumulator = 0;
                    t_move_start = current_time; //Time the saccade started

                } else if ( right && (!avoid) ){ //Right turn required
                    Log.i("CA:", "Right turn triggered");
                    Command.go( new double[]{0, 0});
                    try { sleep(1000); } catch( Exception e ){ e.printStackTrace(); }
                    try {
                        Command.turnAround(-turn);
                    }catch(Exception e){ e.printStackTrace(); }
                    /*lft_speed = 100; //Right turn
                    rgt_speed = 10; //Right turn*/
                    initialise = true; //Re-call the go command to trigger the speed change
                    avoid = true; //Flag so we know to ignore this code snippet when avoiding
                    dual_accumulator = 0;
                    left_accumulator = 0;
                    right_accumulator = 0;
                    t_move_start = current_time; //Time the saccade started

                } else if ( avoid && ((current_time - t_move_start ) >= 500)){
                    //Make adjustments in half second intervals
                    lft_speed = 15; //Back to default
                    rgt_speed = 14; //Back to default
                    avoid = false; //No longer avoiding an obstacle
                    initialise = true; //Need to reset robot speeds
                }

                loop_count++;
                current_time = (int) SystemClock.elapsedRealtime() - start_time; //Update current time
                t_delta = (int) SystemClock.elapsedRealtime() - t_interval_start; //Update accumulation interval
            }

            try {
                main.runOnUiThread(new Runnable() {
                    @Override
                    public void run(){
                        main.debugTextView.setText(String.format(
                                "Obstacle seen"
                        ));
                    }
                });

            } catch (Exception e){
                e.printStackTrace();
            }

            Command.go( new double[] { 0, 0} );
            try{
                sleep( 1000 );
            } catch (Exception e){
                e.printStackTrace();
            }
        }

    };

}
