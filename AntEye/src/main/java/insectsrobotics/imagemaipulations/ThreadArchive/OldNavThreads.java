package insectsrobotics.imagemaipulations.ThreadArchive;

import insectsrobotics.imagemaipulations.*;
import insectsrobotics.imagemaipulations.MainActivity;

import org.opencv.core.Mat;
import org.ejml.simple.SimpleMatrix;

import android.util.Log;
import android.os.SystemClock;

import static java.lang.Thread.sleep;

public class OldNavThreads {
    private MainActivity main;
    //
    // Constructor. For legacy reasons we need access to MainActivity for various global variables
    // This is mostly Zhaoyu/Luca's old code including combiner threads
    // This code is kept purely for legacy reasons and isn't used mostly
    // except for reference purposes
    //
    public OldNavThreads(MainActivity main){
        this.main = main;
    }

    /**
     * start searching
     */
    public Runnable startSearch=new Runnable() {
        @Override
        public void run() {
            for(;;){
                if(main.savedList){
                    main.savedList=false;
                    try {
                        sleep(15000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    main.broadcast.broadcastStartSearch(true);
                }
            }

        }
    };

    public Runnable startScanning = new Runnable() {
        @Override
        public void run() {

            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            int min_index = 0;

            while (true) {
                try {
                    Command.turnAround(30.00);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                Boolean image_not_accessed;

                String unfarmiliarity_distribution = "";

                for (int i = 0; i < 11; i++) {
                    image_not_accessed = true;
                    while (image_not_accessed){
                        if (main.images_access){
                            Mat matImage = main.processedDestImage;
                            //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                            byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                            matImage.get(0, 0, imageArray_tmp);
                            int[] imageArray = new int[imageArray_tmp.length];
                            for (int n = 0; n < imageArray_tmp.length; n++) {
                                imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                            }
                            main.familarity_array[i] = main.model.calculateFamiliarity(imageArray);
                            unfarmiliarity_distribution += main.familarity_array[i] + ",";
                            image_not_accessed = false;
                        }}

                    if (i != 10) {
                        try {
                            Command.turnAround(-6.00);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                }

                LogToFileUtils.write(unfarmiliarity_distribution+'\n');

                min_index = Util.getMinIndex(main.familarity_array);

                LogToFileUtils.write("Seleted index: " + min_index + "\n");

                try {
                    Command.turnAround((10 - min_index) * 6.0);
                } catch (Exception e) {
                    e.printStackTrace();
                }

                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                Command.moveForward(0.10);

                try {
                    sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        }
    };


    public Runnable startKlinokinesis = new Runnable() {
        @Override
        public void run() {

            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            double current_familiarity;
            double maximum_unfamiliarity = main.model.calculateMaximumUnfamiliarity();
            LogToFileUtils.write("Maximum Unfamiliarity: " + maximum_unfamiliarity + "\n");
            double normalised_familiarity;

            // Tuning parameters
            double alpha = 80.0;
            double beta = 0.1;

            // Moving Decision
            double turn_angle;
            double step_size;

            int flag = 1;

            while(true) {
                if (main.images_access){
                    Mat matImage = main.processedDestImage;
                    //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                    byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                    matImage.get(0, 0, imageArray_tmp);
                    int[] imageArray = new int[imageArray_tmp.length];
                    for (int n = 0; n < imageArray_tmp.length; n++) {
                        imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                    }

                    current_familiarity = main.model.calculateFamiliarity(imageArray);
                    LogToFileUtils.write("Current Familiarity: " + current_familiarity+ "\n");

                    normalised_familiarity = current_familiarity / maximum_unfamiliarity;
                    LogToFileUtils.write("Normalised Familiarity: " + normalised_familiarity + "\n");

                    turn_angle = alpha * normalised_familiarity;
                    LogToFileUtils.write("Turn Angle: " + turn_angle + "\n");

                    step_size = (1 - normalised_familiarity) * beta;
                    LogToFileUtils.write("Step Size: " + step_size + "\n");


                    try {
                        Command.turnAround(flag * turn_angle);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    try {
                        sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    Command.moveForward(step_size);

                    try {
                        sleep(2000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    flag = -flag;
                }
            }
        }
    };


    public Runnable startCombiner = new Runnable() {
        @Override
        public void run() {

            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            SimpleMatrix tl2 = new SimpleMatrix(CX_MB.n_tl2, 1);
            tl2.set(0);
            SimpleMatrix cl1 = new SimpleMatrix(CX_MB.n_cl1, 1);
            cl1.set(0);
            SimpleMatrix tb1 = new SimpleMatrix(CX_MB.n_tb1, 1);
            tb1.set(0);
            SimpleMatrix memory = new SimpleMatrix(CX_MB.n_cpu4, 1);
            memory = main.stored_memory.copy();
            SimpleMatrix cpu4 = new SimpleMatrix(CX_MB.n_cpu4, 1);
            cpu4.set(0);
            SimpleMatrix cpu1 = new SimpleMatrix(CX_MB.n_cpu1, 1);
            cpu1.set(0);

            // Zhaoyu added this
            SimpleMatrix en = new SimpleMatrix(CX_MB.n_tb1, 1);
            en.set(0);

            int T_inbound = 30;

            main.startTime = (int) SystemClock.elapsedRealtime();
            int t0 = (int) SystemClock.elapsedRealtime();
            int t = 0;
            main.CURRiteration = 0;
            int currentTime = (int) SystemClock.elapsedRealtime() - t0;

            Boolean image_not_accessed;

            while (currentTime < T_inbound * 1000) {
                main.CURRiteration++;
                try {
                    sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                image_not_accessed = true;
                while (image_not_accessed){
                    if (main.images_access){
                        Mat matImage = main.processedDestImage;
                        //Since there is no possibility to send a 2D array or Mat file the image is transformed to a 1D int array
                        byte[] imageArray_tmp = new byte[matImage.height() * matImage.width()];
                        matImage.get(0, 0, imageArray_tmp);
                        int[] imageArray = new int[imageArray_tmp.length];
                        for (int n = 0; n < imageArray_tmp.length; n++) {
                            imageArray[n] = (int) imageArray_tmp[n] & 0xFF;
                        }
                        en = main.cxmb.calculateFamiliarityDistribution(imageArray);
                        image_not_accessed = false;
                        LogToFileUtils.write("Image Updated");
                    }
                }

                //------   COMPASS UPDATE  -----
                tl2 = main.cxmb.tl2Output(Math.toRadians(main.currentDegree));
                cl1 = main.cxmb.cl1Output(tl2);
                tb1 = main.cxmb.tb1Output(cl1, tb1);

                // ------ DISPLACEMENT UPDATE -----
                memory = main.cxmb.cpu4Update(memory, tb1, main.ANT_SPEED);
                cpu4 = main.cxmb.cpu4Output(memory.copy());

                // ----- TURNING GENERATION ------
                cpu1 = main.cxmb.cpu1Output(tb1, cpu4, en, false);
                //cpu1 = main.cxmb.cpu1EnOutput(tb1, en);
                main.CXmotor = main.cxmb.motorOutput(cpu1);
                //Util.writeToFile(memory, main.leftCXFlow, main.rightCXFlow, "run_1");

                // ----- ANTBOT DRIVING--------
                main.CXnewHeading = Math.toDegrees(Math.toRadians(main.currentDegree) - main.CXmotorChange * main.CXmotor);
                main.CXtheta = (main.CXnewHeading - main.currentDegree) % 360;

                t = (int) SystemClock.elapsedRealtime() - t0;
                String direction = "not sure";
                if (t > 300) {
                    // speed in dm/sec
                    if (main.CXtheta < -1.5) {
                        Command.go(new double[]{10, 100});
                        main.ANT_SPEED = 0.5;
                        direction = "left";
                    } else if (main.CXtheta > 1.5) {
                        Command.go(new double[]{100, 10});
                        main.ANT_SPEED = 0.5;
                        direction = "right";
                    } else {
                        Command.go(new double[]{100, 100});
                        main.ANT_SPEED = 4.0;
                        direction = "straight";
                    }
                    Log.d(main.TAG, "facing " + main.currentDegree + ", going " + direction + "\nwant to go to " + main.CXnewHeading);
                    t0 = (int) SystemClock.elapsedRealtime();
                    Log.d(main.TAG, Util.printMemory(memory));
                }

                try {
                    main.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            main.debugTextView.setText(String.format("inbound " +
                                            "\ncurrent iteration: %d" +
//                                    "\ncurrent direction: %f"+
                                            "\nLeft Speed: %f" +
                                            "\nRight Speed: %f" +
                                            "\nWant to go to: %f" +
                                            "\nSo turning of: %f",
                                    main.CURRiteration,
//                                    main.currentDegree,
                                    main.leftCXFlow,
                                    main.rightCXFlow,
                                    main.CXnewHeading,
                                    main.CXtheta));
                        }
                    });

                } catch (Exception e) {
                    e.printStackTrace();
                }
                if (Util.isHome(memory)) break;
                currentTime = (int) SystemClock.elapsedRealtime() - main.startTime;
            }

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Command.go(new double[]{0, 0});

            try {
                main.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        main.debugTextView.setText(String.format("The PI Run is over! " +
                                "\nTo start again please return to the " +
                                "\nprevious page and click -Start-"));
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };


    /**
     *
     * Start Homing
     */
    public Runnable startHome = new Runnable() {
        @Override
        public void run() {
            for(;;){
                if(main.savedList){
                    main.savedList=false;
                    try {
                        sleep(15000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    main.broadcast.broadcastStartHome(true);
                }
            }
        }
    };

    public Runnable saveImagesRunDown =new Runnable() {
        @Override
        public void run() {
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Mat temp=new Mat();
            for(int i=0;i<360;i++){
                temp=Util.rotateFullImage(main.fullSnapShot,i);
            }
            Log.i(main.flowTag,"Saving Images Done");
            main.savedList=true;
        }
    };

    public Runnable saveImagesTurnMinimum = new Runnable() {
        @Override
        public void run() {
            for(int n=0;n<4;n++){
                Log.i(main.flowTag,"start with list "+(n+1));
                Mat temp=new Mat();
                for(int i=0;i<360;i=i+4){
                    temp=Util.rotateImage(main.imageToDisplay, i);
                }
                Log.i(main.flowTag,"Done with list "+(n+1));
                try {
                    sleep(15000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            Log.i(main.flowTag,"Images saved");
            main.savedList=true;

        }
    };





}
