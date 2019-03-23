package insectsrobotics.imagemaipulations;

import android.util.Log;

import org.ejml.simple.SimpleMatrix;
import insectsrobotics.imagemaipulations.NavigationModules._Superclasses.NavigationModules;
import insectsrobotics.imagemaipulations.LogToFileUtils;

import java.util.Arrays;
import java.util.Random;

//
// Class to act as the Extended Central Complex model which will incorporate the MB for visual
// navigation and some Collision Avoidance mechanism. Framework copied from CX_MB by Zhaoyu Zhang.
//
public class CXE extends NavigationModules {
    //
    // MB Parameters
    //
    private int[][] willshawConnectionArray;
    private byte[][] willshawNetworkWeight;
    private int learnedKCs = 0;
    private boolean networkSetUp = false;
    private boolean firstImage = true;
    private boolean inverse = true;
    private int threshold = 1750;
    private final String TAG = this.getClass().getSimpleName();
    private int current_direction = 0;

    private int learned_images= 0;

    //
    // CA parameters - Algorithmic, and neural model
    //

    private int lft_accumulator = 0;
    private int rgt_accumulator = 0;
    private int call_count = 0;
    private int accumulation_threshold = 250000;
    private int reaction_threshold = 1000000;
    private boolean left_saccade = false;
    private boolean right_saccade = false;

    private SimpleMatrix ca_neurons = new SimpleMatrix(8, 1);
    private SimpleMatrix acc = new SimpleMatrix(2, 1);

    // The accumulation neurons need some leak
    private double acc_leak = 0.1;

    public static boolean collisionDetected = false;


    // ---------------   PARAMETERS  ---------------------
    public static int n_tl2 = 16;
    public static int n_cl1 = 16;
    public static int n_tb1 = 8;
    public static int n_cpu4 = 16;
    public static int n_cpu1 = 16;
    public static int n_acc = 2;
    public static int n_ca = 16;

    // Zhaoyu added this for right ENs
    static int n_en = 8;

    static double tb_tb_weight = 1.;
    SimpleMatrix tl2_prefs_default = new SimpleMatrix(new double[][] {
            {0.},  {0.78539816}, {1.57079633}, {2.35619449},  {3.14159265},
            {3.92699082},  {4.71238898},  {5.49778714},  {0.},  {0.78539816},
            {1.57079633},  {2.35619449},  {3.14159265},  {3.92699082},  {4.71238898},
            {5.49778714}
    });
    double noise_default = 0.0;  // 10% additive noise


    //--------------- TUNED PARAMETERS  ----------------------
    double cpu4_mem_gain_default = 0.005;
    double cpu4_mem_loss_default = 0.0026;  // This is tuned to keep memory constant...

    double tl2_slope_tuned = 6.8;
    double tl2_bias_tuned = 3.0;

    double cl1_slope_tuned = 3.0;
    double cl1_bias_tuned = -0.5;

    double tb1_slope_tuned = 5.0;
    double tb1_bias_tuned = 0;

    double cpu4_slope_tuned = 5.0;
    double cpu4_bias_tuned = 2.5;

    double cpu1_slope_tuned = 6.0;
    double cpu1_bias_tuned = 2.0;
    //-------------------------------------------------------------------------------



    //----------------  CLASS PARAMETERS  ------------------

    double noise;
    double tl2_slope;
    double tl2_bias;
    SimpleMatrix tl2_prefs;
    double cl1_slope;
    double cl1_bias;
    double tb1_slope;
    double tb1_bias;
    double cpu4_slope;
    double cpu4_bias;
    SimpleMatrix cpu4_mem;
    double cpu4_mem_gain;
    double cpu4_mem_loss;
    double cpu1_slope;
    double cpu1_bias;

    //---- anatomical matrices

    SimpleMatrix W_CL1_TB1;
    SimpleMatrix W_TB1_TB1;
    SimpleMatrix W_TB1_CPU1;
    SimpleMatrix W_TB1_CPU4;
    SimpleMatrix W_CPU4_CPU1;
    SimpleMatrix W_CPU1_motor;

    public CXE(){
        super();
        willshawConnectionArray = new int[20000][10];
        Log.e(TAG, "Constructor Called");
        LogToFileUtils.write("Willshaw Module Constructor Called");

        // parameter initialisation
        this.noise = noise_default;
        this.tl2_slope = tl2_slope_tuned;
        this.tl2_bias = tl2_bias_tuned;
        this.tl2_prefs  = tl2_prefs_default;
        this.cl1_slope  = cl1_slope_tuned;
        this.cl1_bias  = cl1_bias_tuned;
        this.tb1_slope  = tb1_slope_tuned;
        this.tb1_bias  = tb1_bias_tuned;
        this.cpu4_slope  = cpu4_slope_tuned;
        this.cpu4_bias  = cpu4_bias_tuned;
        this.cpu4_mem  = new SimpleMatrix(new double[n_cpu4][1]);
        cpu4_mem.set(0);
        this.cpu4_mem_gain  = cpu4_mem_gain_default;
        this.cpu4_mem_loss  = cpu4_mem_loss_default;
        this.cpu1_slope  = cpu1_slope_tuned;
        this.cpu1_bias  = cpu1_bias_tuned;

        // Weight matrices initialisation based on anatomy.
        SimpleMatrix tmp = SimpleMatrix.identity(n_tb1);
        this.W_CL1_TB1 = tmp.combine(
                0, tmp.numCols(), tmp
        );
        this.W_TB1_TB1 = genTbTbWeights(1.);
        this.W_TB1_CPU1 = tmp.combine(
                tmp.numRows(),0, tmp
        );
        this.W_TB1_CPU4 = tmp.combine(
                tmp.numRows(),0, tmp
        );

        this.W_CPU4_CPU1 = new SimpleMatrix(new double[][] {
                {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
                {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
                {0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0},
                {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0}
        }).transpose();
        this.W_CPU1_motor = new SimpleMatrix(new double[][] {{-1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1}});


    }
    // MB Functions
    @Override
    public void setupLearningAlgorithm(int[] image) {
        Log.e(TAG, "setupLearningAlgorithm called");
        LogToFileUtils.write("setupLearningAlgorithm called");
        super.setupLearningAlgorithm(image);
        // Initialise the weight with all 1.
        willshawNetworkWeight = new byte[8][willshawConnectionArray.length];
        for (int i =0; i < 8; i++){
            Arrays.fill(willshawNetworkWeight[i], (byte) 1);
        }

        if (inverse){
            for (int n = 0; n < image.length; n++){
                image[n] = 255 - image[n];
            }
        }

        Random random;
        for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
            for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                random = new Random();
                willshawConnectionArray[KCNumber][PNNumber] = random.nextInt(image.length);
                //Log.e("WillshawModule", "PixelNumber = " + willshawConnectionArray[KCNumber][PNNumber]);
            }
        }

        networkSetUp = true;

    }

    private boolean lowered = false;
    private boolean raised = false;
    private int adjustment = 10;

    @Override
    public void learnImageWithDirection(int[] image, SimpleMatrix tb1) {
        super.learnImage(image);

        double maximum_direction = tb1.get(0,0);
        for (int i = 0; i < 8; i++){
            if (tb1.get(i,0) >= maximum_direction){
                current_direction = i;
            }
        }

        if (firstImage) {
            while (learnedKCs >= 400 || learnedKCs <= 200) {

                for (int i =0; i < 8; i++){
                    Arrays.fill(willshawNetworkWeight[i], (byte) 1);
                }

                learnedKCs = 0;
                for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
                    int brightness = 0;
                    for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                        int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];

                        brightness = brightness + brightness_tmp;
                    }
                    if (brightness >= threshold) {
                        if (willshawNetworkWeight[current_direction][KCNumber] != 0) {
                            willshawNetworkWeight[current_direction][KCNumber] = 0;
                            learnedKCs++;
                        }
                    }
                }
                if (learnedKCs >= 400) {
                    if (lowered) {
                        adjustment--;
                    }
                    threshold = threshold + adjustment;
                    raised = true;
                    lowered = false;
                } else if (learnedKCs <= 200) {
                    if (raised) {
                        adjustment--;
                    }
                    threshold = threshold - adjustment;
                    lowered = true;
                    raised = false;
                }
                LogToFileUtils.write("Threshold after adjustment = " + threshold);
                Log.e(TAG, "Threshold after adjustment = " + threshold);
            }

            firstImage = false;

        } else {
            learnedKCs = 0;
            for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
                int brightness = 0;
                for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                    int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];
                    brightness = brightness + brightness_tmp;
                }
                if (brightness >= threshold) {
                    if (willshawNetworkWeight[current_direction][KCNumber] != 0) {
                        willshawNetworkWeight[current_direction][KCNumber] = 0;
                        learnedKCs++;
                    }
                }
            }
        }
        //LogToFileUtils.write("Direction: " + current_direction);
        LogToFileUtils.write("Deactivated Kenyon Cells: " + learnedKCs);
        learned_images += 1;
        LogToFileUtils.write("Total Images: " + learned_images);
        learnedKCs = 0;
    }

    @Override
    public double calculateFamiliarity(int[] image) {
        super.calculateFamiliarity(image);
        int sum = 0;
        double error[] = new double[8];
        if (networkSetUp) {
            for (int i = 0; i < 8; i++){
//                for (int i : willshawNetworkWeight[current_direction]) {
//                    sum += i;
//                }
//                Log.e(TAG, "Array Sum: " + sum);
                Log.e(TAG, "Threshold: " + threshold);
                for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
                    int brightness = 0;
                    for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                        int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];
                        brightness = brightness + brightness_tmp;
                    }
                    if (brightness >= threshold) {
                        error[i] = error[i] + willshawNetworkWeight[i][KCNumber];
                    }
                }
                LogToFileUtils.write("Calculated Error: " + error);
                Log.e(TAG, "Error: " + error);
            }

        }
        return 0;
    }

    public SimpleMatrix calculateFamiliarityDistribution(int[] image){

        double error[] = new double[8];

        super.calculateFamiliarity(image);

        if (networkSetUp) {
            for (int i = 0; i < 8; i++){
                for (int KCNumber = 0; KCNumber < willshawConnectionArray.length; KCNumber++) {
                    int brightness = 0;
                    for (int PNNumber = 0; PNNumber < 10; PNNumber++) {
                        int brightness_tmp = image[willshawConnectionArray[KCNumber][PNNumber]];
                        brightness = brightness + brightness_tmp;
                    }
                    if (brightness >= threshold) {
                        error[i] = error[i] + willshawNetworkWeight[i][KCNumber];
                    }
                }
                //LogToFileUtils.write("Calculated Error: " + error[i]);
                //Log.e(TAG, "Error: " + error);
            }

        }

        // String result_str = "";
        // String error_str = "";
        SimpleMatrix normalised_result = new SimpleMatrix(n_en,1);
        double error_sum = 0;

        for (int i = 0; i < 8; i++){
            error_sum += error[i];
            //error_str += error[i] + " ";
        }
        for (int i = 0; i < 8; i++){
            normalised_result.set(i, 0, 1 - error[i]/error_sum);
        }

//        for (int i = 0; i < 8; i++){
//            normalised_result.set(i, 0, 1 / (1 + Math.exp(-normalised_result.get(i,0))));
//            result_str +=  error[i]/error_sum + ",";
//        }

        //LogToFileUtils.write("Calculated Error: " + result_str);
        return normalised_result;
    }

//    public double calculateMaximumUnfamiliarity() {
//        return maximum_KCs;
//    }


    //--------------- GETTERS AND SETTERS ----------------------
    public double getNoise() {
        return noise;
    }

    public void setNoise(double noise) {
        this.noise = noise;
    }

    public double getTl2_slope() {
        return tl2_slope;
    }

    public void setTl2_slope(double tl2_slope) {
        this.tl2_slope = tl2_slope;
    }

    public double getTl2_bias() {
        return tl2_bias;
    }

    public void setTl2_bias(double tl2_bias) {
        this.tl2_bias = tl2_bias;
    }

    public SimpleMatrix getTl2_prefs() {
        return tl2_prefs;
    }

    public void setTl2_prefs(SimpleMatrix tl2_prefs) {
        this.tl2_prefs = tl2_prefs;
    }

    public double getCl1_slope() {
        return cl1_slope;
    }

    public void setCl1_slope(double cl1_slope) {
        this.cl1_slope = cl1_slope;
    }

    public double getCl1_bias() {
        return cl1_bias;
    }

    public void setCl1_bias(double cl1_bias) {
        this.cl1_bias = cl1_bias;
    }

    public double getTb1_slope() {
        return tb1_slope;
    }

    public void setTb1_slope(double tb1_slope) {
        this.tb1_slope = tb1_slope;
    }

    public double getCpu4_slope() {
        return cpu4_slope;
    }

    public void setCpu4_slope(double cpu4_slope) {
        this.cpu4_slope = cpu4_slope;
    }

    public double getTb1_bias() {
        return tb1_bias;
    }

    public void setTb1_bias(double tb1_bias) {
        this.tb1_bias = tb1_bias;
    }

    public double getCpu4_bias() {
        return cpu4_bias;
    }

    public void setCpu4_bias(double cpu4_bias) {
        this.cpu4_bias = cpu4_bias;
    }

    public SimpleMatrix getCpu4_mem() {
        return cpu4_mem;
    }

    public void setCpu4_mem(SimpleMatrix cpu4_mem) {
        this.cpu4_mem = cpu4_mem;
    }

    public double getCpu4_mem_gain() {
        return cpu4_mem_gain;
    }

    public void setCpu4_mem_gain(double cpu4_mem_gain) {
        this.cpu4_mem_gain = cpu4_mem_gain;
    }

    public double getCpu4_mem_loss() {
        return cpu4_mem_loss;
    }

    public void setCpu4_mem_loss(double cpu4_mem_loss) {
        this.cpu4_mem_loss = cpu4_mem_loss;
    }

    public double getCpu1_slope() {
        return cpu1_slope;
    }

    public void setCpu1_slope(double cpu1_slope) {
        this.cpu1_slope = cpu1_slope;
    }

    public double getCpu1_bias() {
        return cpu1_bias;
    }

    public void setCpu1_bias(double cpu1_bias) {
        this.cpu1_bias = cpu1_bias;
    }

    public SimpleMatrix getW_CL1_TB1() {
        return W_CL1_TB1;
    }

    public void setW_CL1_TB1(SimpleMatrix w_CL1_TB1) {
        W_CL1_TB1 = w_CL1_TB1;
    }

    public SimpleMatrix getW_TB1_TB1() {
        return W_TB1_TB1;
    }

    public void setW_TB1_TB1(SimpleMatrix w_TB1_TB1) {
        W_TB1_TB1 = w_TB1_TB1;
    }

    public SimpleMatrix getW_TB1_CPU1() {
        return W_TB1_CPU1;
    }

    public void setW_TB1_CPU1(SimpleMatrix w_TB1_CPU1) {
        W_TB1_CPU1 = w_TB1_CPU1;
    }

    public SimpleMatrix getW_TB1_CPU4() {
        return W_TB1_CPU4;
    }

    public void setW_TB1_CPU4(SimpleMatrix w_TB1_CPU4) {
        W_TB1_CPU4 = w_TB1_CPU4;
    }

    public SimpleMatrix getW_CPU4_CPU1() {
        return W_CPU4_CPU1;
    }

    public void setW_CPU4_CPU1(SimpleMatrix w_CPU4_CPU1) {
        W_CPU4_CPU1 = w_CPU4_CPU1;
    }

    public SimpleMatrix getW_CPU1_motor() {
        return W_CPU1_motor;
    }

    public void setW_CPU1_motor(SimpleMatrix w_CPU1_motor) {
        W_CPU1_motor = w_CPU1_motor;
    }

    //-------------------------------------------------------------------------------------
    // -------------------- private methods for internal use ------------------------------
    private SimpleMatrix genTbTbWeights (double weight) {
        SimpleMatrix W = new SimpleMatrix(n_tb1, n_tb1);
        double[] sinusoid = new double[]{-0., 0.14644661, 0.5, 0.85355339, 1., 0.85355339, 0.5, 0.14644661};
        for (int i=0; i<n_tb1; i++) {
            for (int j=0; j<n_tb1; j++) {
                int ind = (j-i)%(n_tb1);
                if (ind<0) {
                    ind = 8+ind;
                }
                W.set(i, j, sinusoid[ind]*weight);
            }
        }
        return W;
    }

    public static SimpleMatrix noisySigmoid(SimpleMatrix v, double slope, double bias, double noise){
        //Takes a vector v as input, puts through sigmoid //
        for (int i=0; i<v.numRows(); i++){
            v.set(i,0,(1/(1+Math.exp(-(v.get(i,0)*slope - bias)))));
        }

        if (noise > 0) {
            Random r = new Random();
            for (int i=0; i<v.numRows(); i++){
                v.set(i,0,v.get(i,0)+r.nextGaussian()*noise);
            }
        }
        for (int i=0; i<v.numRows(); i++){
            if (v.get(i,0) > 1){
                v.set(i,0,1);
            } else if (v.get(i,0) < 0){
                v.set(i,0,0);
            }
        }
        return v;
    }

    //--------------------------------------------------------------------------------
    //-----------public methods for class interface-----------------------------------

    public SimpleMatrix tl2Output (Double theta){
        // dot product with preferred angle and current heading
        SimpleMatrix output = new SimpleMatrix(this.tl2_prefs.numRows(),1);
        output.set(theta);
        output = output.minus(this.tl2_prefs);
        for(int i=0; i<output.numRows(); i++){
            output.set(i, 0, Math.cos(output.get(i)));
        }
        return noisySigmoid(output, this.tl2_slope, this.tl2_bias, this.noise);
    }

    public SimpleMatrix cl1Output (SimpleMatrix tl2){
        // Takes input from the TL2 neurons and gives output.
        return noisySigmoid(tl2.negative(), this.cl1_slope, this.cl1_bias, this.noise);
    }

    public SimpleMatrix tb1Output(SimpleMatrix cl1, SimpleMatrix tb1){
        // Ring attractor state on the protocerebral bridge.
        Double prop_cl1 = 0.667;
        Double prop_tb1 = 1.0 - prop_cl1;
        SimpleMatrix output = CXE.dot(this.W_CL1_TB1, cl1).scale(prop_cl1).minus(CXE.dot(this.W_TB1_TB1, tb1).scale(prop_tb1));
        return noisySigmoid(output, this.tb1_slope, this.tb1_bias, this.noise);
    }

    public SimpleMatrix cpu4Update (SimpleMatrix cpu4_mem, SimpleMatrix tb1, Double speed){
        SimpleMatrix ones = new SimpleMatrix(tb1.numRows(), 1);
        ones.set(1);
        SimpleMatrix diffMatrix = new SimpleMatrix(cpu4_mem.numRows(), cpu4_mem.numCols());
        diffMatrix.set(speed*this.cpu4_mem_loss);
        cpu4_mem = cpu4_mem.minus(diffMatrix);
        diffMatrix.set(CXE.dot(this.W_TB1_CPU4, ones.minus(tb1)).scale(speed*this.cpu4_mem_gain));
        cpu4_mem = cpu4_mem.plus(diffMatrix);
        for (int i=0; i<cpu4_mem.numRows(); i++){
            if (cpu4_mem.get(i,0) > 1){
                cpu4_mem.set(i,0,1);
            } else if (cpu4_mem.get(i,0) < 0){
                cpu4_mem.set(i,0,0);
            }
        }
        return cpu4_mem;
    }

    public SimpleMatrix cpu4Output (SimpleMatrix cpu4_mem){
        return noisySigmoid(cpu4_mem, this.cpu4_slope, this.cpu4_bias, this.noise);
    }

    public SimpleMatrix accUpdate(double leftSum, double rightSum){
        double flowDiff = leftSum - rightSum;
        double l_val = acc.get(0,0);
        double r_val = acc.get(1,0);

        leftSum /= 1000000;
        rightSum /= 1000000;

        if (call_count > 2){
            // Old method, clear accumulators every 2 calls
            acc.set(0);
            call_count = 0;
        }

        //
        // Accumulate if difference is great enough
        //
        if (flowDiff >= accumulation_threshold){
            l_val += (int) flowDiff;
        } else if (flowDiff <= -(accumulation_threshold)) {
            r_val += (int) Math.abs(flowDiff);
        }
        Log.i("CXE", "flowDiff: " + flowDiff);


        //
        // Update accumulator after leak
        //
        //l_val *= acc_leak;
        //r_val *= acc_leak;

        acc.set(0, 0, leftSum - rightSum);
        acc.set(1, 0, rightSum - leftSum);


        call_count++;


        //Each neuron's activity is the rate (sigmoid) of the input, then inhibited by the
        // opposite side
        SimpleMatrix ret = noisySigmoid(acc, 4, 0, this.noise);
     /*   SimpleMatrix inh = new SimpleMatrix(2,1);
        Log.i("CXE", "(lval, rval): (" + ret.get(0,0) + ", " + ret.get(1,0) + ")");
        //Each side inhibits the other
        inh.set(0,0, -ret.get(1,0));
        inh.set(1,0, -ret.get(0,0));
        Log.i("CXE", "(lval, rval): (" + inh.get(0,0) + ", " + inh .get(1,0) + ")");
        //Apply inhibition
        ret = ret.plus(inh);
        ret = ret.scale(300);*/
        Log.i("CXE", "ret (lval, rval): (" + ret.get(0,0) + ", " + ret.get(1,0) + ")");



        return ret; //noisySigmoid(acc, 10, 0, this.noise);
    }

    public SimpleMatrix caOutput(SimpleMatrix tb1, SimpleMatrix acc) {
        // Offset, how far the sinusoid needs shifted to match the desired heading
        //int offset = 0;

        //
        // Check to see which neurons have fired. If either side fires,
        // clear the acc neurons. If both have fired, we don't want to
        // react. Surely this points to more than a matched filtering system.
        //
        /*
        if (acc.get(0,0) > reaction_threshold &&
                acc.get(1,0) > reaction_threshold) {
            acc.set(0);
            offset = 0;
            CXE.collisionDetected = false;
        } else if (acc.get(0,0) > reaction_threshold){
            acc.set(0);
            offset = -2;
            CXE.collisionDetected = true;
            left_saccade = true;
        } else if ( acc.get(1,0) > reaction_threshold){
            acc.set(0);
            offset = 2;
            CXE.collisionDetected = true;
            right_saccade = true;
        }
        Log.i("CXE", "Offset: " + offset);
*/

        Log.i("CXE", "(lval, rval): (" + acc.get(0,0) + ", " + acc.get(1,0) + ")");

        int offset = (acc.get(0,0) > acc.get(1,0)) ?
                (int) (4*(acc.get(0,0) - 0.5)) :
                (int) -(4*(acc.get(1,0) - 0.5));

        Log.i("CXE", "Offset: " + offset);
        Log.i("CXE", "Offset: " + 4*(acc.get(0,0) - 0.5));
        // Determine current direction
        //
        int current_heading = 0;
        for (int i = 0; i < n_tb1; i++){
            if (tb1.get(i, 0) > tb1.get(current_heading, 0)){
                current_heading = i;
            }
        }

        //Define a sin curve from pi/2 to pi/4; shifted sine curve
        double[] sinusoid = {
                Math.sin(Math.PI / 2),
                Math.sin(3 * Math.PI / 4),
                0,
                Math.sin(5 * Math.PI / 4),
                Math.sin(3 * Math.PI / 2),
                Math.sin(7 * Math.PI / 4),
                0,
                Math.sin(Math.PI / 4)
        };

        //
        // Store the shifted sinusoid and compress it into y=[0,1]
        //
        String out = "";
        SimpleMatrix output = new SimpleMatrix(n_ca, 1);
        SimpleMatrix halfOutput = new SimpleMatrix(n_tb1, 1);
        for (int i = 0; i < n_tb1; i++){
            int s_idx = Util.mod((i + offset + current_heading), n_tb1);
            //int s_idx = Util.mod((i + offset), n_tb1);
            halfOutput.set(s_idx,0, (sinusoid[i]));
            Log.i("CXE", "i: " + i + ", shift: " + s_idx);
        }

        //
        // Interleave results (extend to 16x1 by entering each element twice)
        //

        for ( int i = 0; i < n_ca/2; i++) {
            output.set(i, 0, halfOutput.get(i,0));
            output.set(i + 8, 0, halfOutput.get(i,0));
        }


        //
        // DEBUG: Sinusoid check
        //
        String outstring = "";
        for ( int i = 0; i < n_ca; i++ ){
            outstring = outstring + output.get(i,0) + " ";
        }

        Log.i("CXE", "Current " + current_heading + " : " + outstring);
        return noisySigmoid(output, this.cpu4_slope, this.cpu4_bias, this.noise);
    }

    //
    // I want to add CA capabilities here.
    //
    private boolean accumulateFlow(double leftSum, double rightSum){
        double flowDiff = leftSum - rightSum;
        Log.i("CXE", "call_count: " + call_count );
        //
        // Reset the accumulators every two calls
        //
        if (call_count >= 2){
            lft_accumulator = 0;
            rgt_accumulator = 0;
            call_count = 0;
        }

        Log.i("CXE", "Accumulators: (" + lft_accumulator + ", " + rgt_accumulator + ")");
        Log.i("CXE", "Sums: (" + leftSum + ", " + rightSum + ", " + flowDiff  + ")");
        Log.i("CXE", "Diff: " + flowDiff);


        //
        // Accumulate if difference is great enough
        //
        if (flowDiff >= accumulation_threshold){
            lft_accumulator = lft_accumulator + (int) leftSum;
        } else if (flowDiff <= -(accumulation_threshold)) {
            rgt_accumulator = rgt_accumulator + (int) Math.abs(rightSum);
        }
        Log.i("CXE", "Accumulators: (" + lft_accumulator + ", " + rgt_accumulator + ")");
        //
        // Check the accumulator
        //
        if (lft_accumulator >= reaction_threshold){
            left_saccade = true;
        } else if (rgt_accumulator >= reaction_threshold){
            right_saccade = true;
        }

        // Track the number of calls
        call_count++;


        Log.i("CXE", "Return: " + (left_saccade || right_saccade));
        // Return true if we need to saccade
        return left_saccade || right_saccade;
    }

    //
    // Compute cpu1 output with CA information included
    //
    public SimpleMatrix cpu1OutputCA (
            SimpleMatrix tb1,
            SimpleMatrix cpu4,
            SimpleMatrix en,
            double leftFilterSum,
            double rightFilterSum,
            Boolean training
    ){
        SimpleMatrix weighted_cpu4 = CXE.dot(this.W_CPU4_CPU1, cpu4);
        SimpleMatrix weighted_en = CXE.dot(this.W_TB1_CPU1, en);
        SimpleMatrix combiner;

        collisionDetected = accumulateFlow(leftFilterSum, rightFilterSum);
        Log.i("CXE", "CXE.collisionDetected: " + collisionDetected);
        if (training){
            combiner = weighted_cpu4.plus(weighted_en);
        } else{
            combiner = weighted_cpu4.divide(1.25).plus(weighted_en.divide(5));
            String cpu4_str = "";
            for (int i =0; i<weighted_cpu4.numRows(); i++){
                cpu4_str += weighted_cpu4.get(i, 0)+ ",";
            }
            LogToFileUtils.write("CPU4: " + cpu4_str);

            String en_str = "";
            for (int i =0; i<weighted_en.numRows(); i++){
                en_str += weighted_en.get(i, 0)+ ",";
            }
            LogToFileUtils.write("EN: " + en_str);

            String c_str = "";
            for (int i =0; i<combiner.numRows(); i++){
                c_str += combiner.get(i, 0)+ ",";
            }
            LogToFileUtils.write("Combiner: " + c_str);
        }

        // This removes the Mushroom Body from this model for just now
        SimpleMatrix inputs = CXE.dot(this.W_CPU4_CPU1, cpu4).minus((CXE.dot(this.W_TB1_CPU1, tb1)));

        // Include the Mushroom Body output in the overall output
        //SimpleMatrix inputs = combiner.minus((CXE.dot(this.W_TB1_CPU1, tb1)));

        // For testing Stored Memory Only
        // SimpleMatrix inputs = CXE.dot(this.W_CPU4_CPU1, cpu4).minus((CXE.dot(this.W_TB1_CPU1, tb1)));
        return noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);
    }

    /* Original function definition
    public SimpleMatrix cpu1Output (SimpleMatrix tb1, SimpleMatrix cpu4, SimpleMatrix en, Boolean training){

        SimpleMatrix weighted_cpu4 = CXE.dot(this.W_CPU4_CPU1, cpu4);
        SimpleMatrix weighted_en = CXE.dot(this.W_TB1_CPU1, en);
        SimpleMatrix combiner;

        if (training){
            combiner = weighted_cpu4.plus(weighted_en);
        } else{
            combiner = weighted_cpu4.divide(1.25).plus(weighted_en.divide(5));
            String cpu4_str = "";
            for (int i =0; i<weighted_cpu4.numRows(); i++){
                cpu4_str += weighted_cpu4.get(i, 0)+ ",";
            }
            LogToFileUtils.write("CPU4: " + cpu4_str);

            String en_str = "";
            for (int i =0; i<weighted_en.numRows(); i++){
                en_str += weighted_en.get(i, 0)+ ",";
            }
            LogToFileUtils.write("EN: " + en_str);

            String c_str = "";
            for (int i =0; i<combiner.numRows(); i++){
                c_str += combiner.get(i, 0)+ ",";
            }
            LogToFileUtils.write("Combiner: " + c_str);
        }


        SimpleMatrix inputs = combiner.minus((CXE.dot(this.W_TB1_CPU1, tb1)));

        // For testing Stored Memory Only
        // SimpleMatrix inputs = CXE.dot(this.W_CPU4_CPU1, cpu4).minus((CXE.dot(this.W_TB1_CPU1, tb1)));
        return noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);
    }*/

    public SimpleMatrix cpu1Output (
            SimpleMatrix tb1,
            SimpleMatrix cpu4,
            SimpleMatrix en,
            SimpleMatrix ca,
            int mode){

        SimpleMatrix weighted_cpu4 = CXE.dot(this.W_CPU4_CPU1, cpu4);
        SimpleMatrix weighted_en = CXE.dot(this.W_TB1_CPU1, en);
        SimpleMatrix weighted_ca = CXE.dot(this.W_CPU4_CPU1, ca);
        SimpleMatrix combiner;

        // Collision avoidance precedence in PI + CA mode
        double ca_weight = 0.5;

        if (mode == 0){
            //combiner = weighted_cpu4.plus(weighted_ca);
            combiner = weighted_cpu4.scale(1 - ca_weight).plus(weighted_ca.scale(ca_weight));
            //combiner = weighted_cpu4.plus(weighted_en);
        } else if (mode == 1) {
            // Just CA, for some reason we need to include cpu4 or we get no reaction.
            combiner = weighted_ca; //weighted_cpu4.plus(weighted_ca);
        } else {
            combiner = weighted_cpu4.divide(1.25).plus(weighted_en.divide(5));
            String cpu4_str = "";
            for (int i =0; i<weighted_cpu4.numRows(); i++){
                cpu4_str += weighted_cpu4.get(i, 0)+ ",";
            }
            LogToFileUtils.write("CPU4: " + cpu4_str);

            String en_str = "";
            for (int i =0; i<weighted_en.numRows(); i++){
                en_str += weighted_en.get(i, 0)+ ",";
            }
            LogToFileUtils.write("EN: " + en_str);

            String c_str = "";
            for (int i =0; i<combiner.numRows(); i++){
                c_str += combiner.get(i, 0)+ ",";
            }
            LogToFileUtils.write("Combiner: " + c_str);
        }

        String t_str = "";
        for(int i = 0; i < tb1.numRows(); i++)
            t_str += tb1.get(i, 0) + " ";

        Log.i("CXE", "TB1 bef: " + t_str);

        SimpleMatrix w_tb1 = CXE.dot(this.W_TB1_CPU1, tb1);
        t_str = "";
        for(int i = 0; i < w_tb1.numRows(); i++)
            t_str += w_tb1.get(i, 0) + " ";

        Log.i("CXE", "TB1 aft: " + t_str);

        SimpleMatrix inputs = combiner.minus((CXE.dot(this.W_TB1_CPU1, tb1)));
        SimpleMatrix out = noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);

        //
        // DEBUG: Compute output strings
        //
        String ca_string = "";
        String cpu4_string = "";
        String combiner_string = "";
        String i_str = "";
        String o_str = "";
        for (int i = 0; i < (2*n_tb1); i++){
            ca_string = ca_string + weighted_ca.get(i, 0) + ", ";
            cpu4_string = cpu4_string + weighted_cpu4.get(i, 0) + ", ";
            combiner_string = combiner_string + combiner.get(i, 0) + ", ";
            i_str += inputs.get(i, 0) + ", ";
            o_str += out.get(i,0) + ", ";
        }
        Log.i("CXE", "WCA: " + ca_string);
        Log.i("CXE", "CPU4: " + cpu4_string);
        Log.i("CXE", "COM: "+ combiner_string);
        Log.i("CXE", "TBI: "+ i_str);
        Log.i("CXE", "TBO: "+ o_str);


        // For testing Stored Memory Only
        // SimpleMatrix inputs = CXE.dot(this.W_CPU4_CPU1, cpu4).minus((CXE.dot(this.W_TB1_CPU1, tb1)));
        return noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);
    }

    public SimpleMatrix cpu1EnOutput (SimpleMatrix tb1, SimpleMatrix en){

        SimpleMatrix inputs = CXE.dot(this.W_TB1_CPU1, en).minus((CXE.dot(this.W_TB1_CPU1, tb1)));

        return noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);
    }

    public Double motorOutput(SimpleMatrix cpu1){
        double saccade_angle = 20.0;
        double output = 0;
        //
        // If a collision avoidance saccade is required, overwrite any CX output and send turn info
        // in the required direction.
       //
        if (left_saccade){
            left_saccade = false;
            right_saccade = false;
            lft_accumulator = 0;
            rgt_accumulator = 0;
            //collisionDetected = false;

//            output = saccade_angle;
        } else if (right_saccade) {
            left_saccade = false;
            right_saccade = false;
            lft_accumulator = 0;
            rgt_accumulator = 0;
            //collisionDetected = false;

//            output = -(saccade_angle);
        }

        output =  this.W_CPU1_motor.dot(cpu1);
        Log.i("CXE", "Output: " + output);
        return output;
        //return this.W_CPU1_motor.dot(cpu1);
    }


    //------------------------- Helper custom Matrix methods --------------------------
    //---------------------------------------------------------------------------------
    public static SimpleMatrix insertInColumn(SimpleMatrix M, SimpleMatrix c, Integer index){
        if (c.numCols() > 1){
            if(c.numRows() == 1){
                c.transpose();
            } else {
                throw new IllegalStateException();
            }
        }

        for (int i=0; i<c.numRows(); i++){
            M.set(i, index, c.get(i,0));
        }
        return M;
    }

    public static SimpleMatrix insertInRow(SimpleMatrix M, SimpleMatrix c, Integer index){
        return insertInColumn(M.transpose(), c.transpose(), index).transpose();
    }

    public static SimpleMatrix dot(SimpleMatrix M, SimpleMatrix T){
        SimpleMatrix R = new SimpleMatrix(M.numRows(), T.numCols());
        for(int i=0; i<M.numRows(); i++){
            for(int j=0; j<T.numCols(); j++){
                R.set(i,j,M.extractVector(true, i).dot(T.extractVector(false, j)));
            }
        }
        return R;
    }

    public static SimpleMatrix clip(SimpleMatrix M, double lower_bound, double upper_bound) {
        for (int i=0; i<M.numRows(); i++){
            for (int j=0; j<M.numCols(); j++) {
                if (M.get(i,j) < lower_bound) {
                    M.set(i,j, lower_bound);
                } else if (M.get(i,j) > upper_bound) {
                    M.set(i,j, upper_bound);
                }
            }
        }
        return M;
    }
}
