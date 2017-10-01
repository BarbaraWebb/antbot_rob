package insectsrobotics.imagemaipulations;

import org.ejml.simple.SimpleMatrix;

import java.util.Random;
import org.jtransforms.fft.DoubleFFT_1D;
/**
 * Created by Luca on 04-Oct-16.
 * Code adapted from python path integrator in Stone et all
 */

public class CX_H_Pontin extends CX_Holonomic {

    private double pontin_slope;
	private double pontin_bias;
	private SimpleMatrix W_pontin_CPU1_A;
	private SimpleMatrix W_pontin_CPU1_B;
	private SimpleMatrix W_CPU4_pontin;

	//---------------------------------------------------------------------------------------

    

    public CX_H_Pontin(){
        // parameter initialisation
    	super();
        this.cpu4_mem_gain *= 0.5;
        this.cpu1_bias = -1.0;
        this.cpu1_slope = 7.5;
        
        //pontine cells
        this.pontin_slope = 5.0;
        this.pontin_bias = 2.5;
        
        this.W_pontin_CPU1_A = new SimpleMatrix(new double [][]{
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}, 
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                 {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
                 {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                 {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
        });
        this.W_pontin_CPU1_B = new SimpleMatrix(new double[][] {
        	{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}
        });
        
        this.W_CPU4_pontin = SimpleMatrix.identity(this.n_cpu4);

        
    }


    public SimpleMatrix cpu4Update (SimpleMatrix cpu4_mem, SimpleMatrix tb1, SimpleMatrix tn1, SimpleMatrix tn2){
        
    	SimpleMatrix diff = new SimpleMatrix(cpu4_mem.numRows(), cpu4_mem.numCols());
        diff.set(0.125*this.cpu4_mem_gain);
        
        SimpleMatrix mem_update = CX.dot(this.W_TN_CPU4, tn2);
        mem_update = mem_update.minus(CX.dot(this.W_TB1_CPU4, tb1));
        mem_update = CX.clip(mem_update, 0.0, 1.0);
        mem_update = mem_update.scale(this.cpu4_mem_gain);
        cpu4_mem = cpu4_mem.plus(mem_update);
        
        cpu4_mem = cpu4_mem.minus(diff);

        return CX.clip(cpu4_mem, 0.0, 1.0);
    }

    public SimpleMatrix pontinOutput(SimpleMatrix cpu4){
    	SimpleMatrix inputs = CX.dot(W_CPU4_pontin, cpu4);
    	return CX.noisySigmoid(inputs, this.pontin_slope, this.pontin_bias, this.noise);
    }
    
    public SimpleMatrix cpu1Output_A (SimpleMatrix tb1, SimpleMatrix cpu4){
        SimpleMatrix inputs = CX.dot(this.W_CPU4_CPU1_A, cpu4).scale(0.5);
        SimpleMatrix pontin = this.pontinOutput(cpu4).scale(0.5);
        inputs = inputs.minus(CX.dot(this.W_pontin_CPU1_A,  pontin));
        inputs = inputs.minus(CX.dot(this.W_TB1_CPU1_A, tb1));
        return noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);
    }

    public SimpleMatrix cpu1Output_B (SimpleMatrix tb1, SimpleMatrix cpu4){
        SimpleMatrix inputs = CX.dot(this.W_CPU4_CPU1_B, cpu4).scale(0.5);
        SimpleMatrix pontin = this.pontinOutput(cpu4).scale(0.5);
        inputs = inputs.minus(CX.dot(this.W_pontin_CPU1_B,  pontin));
        inputs = inputs.minus(CX.dot(this.W_TB1_CPU1_B, tb1));
        return noisySigmoid(inputs, this.cpu1_slope, this.cpu1_bias, this.noise);
    }
    
    public double decode_cpu4(SimpleMatrix cpu4){
    	SimpleMatrix first_cell_set = cpu4.extractMatrix(0, cpu4.numRows()/2, 0, 1);
    	// shift right first set of cells
    	double tmp; 
    	tmp = first_cell_set.get(0, 0);
    	for (int i=0; i<first_cell_set.numRows()-1; i++){
    		first_cell_set.set(i+1, 0, first_cell_set.get(i, 0));
    	}
    	first_cell_set.set(0, 0, tmp);
    	
    	SimpleMatrix second_cell_set = cpu4.extractMatrix(cpu4.numRows()/2, cpu4.numRows(), 0, 1);
    	// shift left second set of cells
    	tmp = second_cell_set.get(0, 0);
    	for (int i=0; i<second_cell_set.numRows()-1; i++){
    		second_cell_set.set(i, 0, second_cell_set.get(i+1, 0));
    	}
    	second_cell_set.set(second_cell_set.numRows()-1, 0, tmp);
    	
    	// sum cells
    	SimpleMatrix cpu4_reshaped = first_cell_set.combine(0, 1, second_cell_set).transpose(); 
    	double[] signal = new double[cpu4_reshaped.numCols()];
    	for (int i=0; i<signal.length; i++){
    		signal[i] = cpu4_reshaped.get(0, i)+cpu4_reshaped.get(1,i);
    	}
    	
    	// compute angle from fourier transform
        DoubleFFT_1D fftDo = new DoubleFFT_1D(signal.length);
        double[] fft = new double[signal.length*2];
        System.arraycopy(signal, 0, fft, 0, signal.length);
        fftDo.realForwardFull(fft);
//        for (double i:fft){
//        	System.out.println(i);
//        }
        double[] freq = new double[] {fft[2], -fft[3]};
        double angle = -Math.atan2(freq[1], freq[0]);
    	
		return angle;
    }

}
