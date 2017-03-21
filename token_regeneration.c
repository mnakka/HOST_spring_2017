// ========================================================================================================
// ========================================================================================================
// ***************************************** token_enrollment.c *******************************************
// ========================================================================================================
// ========================================================================================================

#include "common.h"
#include "token_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <math.h>
// ========================================================================================================
// ========================================================================================================
// Start the PUF engine with the current vector to get the timing values. Once it finishes, fetch each of 
// the timing values.
//===========================================================================================================
//===========================================================================================================
uint16_t LFSR_11_A_bits_low(int load_seed, uint16_t seed)
 {
 static uint16_t lfsr;
 uint16_t bit, nor_bit;
/* Load the seed on the first iteration */
 if ( load_seed == 1 )
 lfsr = seed;
 else
 {
/* Allow all zero state. See my BIST class notes in VLSI Testing. Note, we
use low order bits here because bit is shifted onto the low side, not high
side as in my lecture slides. */
 if ( !( (((lfsr >> 9) & 1) == 1) || (((lfsr >> 8) & 1) == 1) ||
(((lfsr >> 7) & 1) == 1) || (((lfsr >> 6) & 1) == 1) || (((lfsr >> 5) & 1) == 1) ||
(((lfsr >> 4) & 1) == 1) || (((lfsr >> 3) & 1) == 1) || (((lfsr >> 2) & 1) == 1) ||
 (((lfsr >> 1) & 1) == 1) || (((lfsr >> 0) & 1) == 1) ) )
 nor_bit = 1;
 else
 nor_bit = 0;
/* xor_out := rand(10) xor rand(8); */
 bit = ((lfsr >> 10) & 1) ^ ((lfsr >> 8) & 1) ^ nor_bit;
/* Change the shift of the bit to match the width of the data type. */
 lfsr = ((lfsr << 1) | bit) & 2047;
 }
 return lfsr;
 }
//===========================================================================================================
//===========================================================================================================
uint16_t LFSR_11_A_bits_high(int load_seed, uint16_t seed)
 {
 static uint16_t lfsr;
 uint16_t bit, nor_bit;
/* Load the seed on the first iteration */
 if ( load_seed == 1 )
 lfsr = seed;
 else
 {
/* Allow all zero state. See my BIST class notes in VLSI Testing. Note, we
use low order bits here because bit is shifted onto the low side, not high
side as in my lecture slides. */
 if ( !( (((lfsr >> 9) & 1) == 1) || (((lfsr >> 8) & 1) == 1) ||
(((lfsr >> 7) & 1) == 1) || (((lfsr >> 6) & 1) == 1) || (((lfsr >> 5) & 1) == 1) ||
(((lfsr >> 4) & 1) == 1) || (((lfsr >> 3) & 1) == 1) || (((lfsr >> 2) & 1) == 1) ||
 (((lfsr >> 1) & 1) == 1) || (((lfsr >> 0) & 1) == 1) ) )
 nor_bit = 1;
 else
 nor_bit = 0;
/* xor_out := rand(10) xor rand(8); */
 bit = ((lfsr >> 10) & 1) ^ ((lfsr >> 8) & 1) ^ nor_bit;
 lfsr = ((lfsr << 1) | bit) & 2047;
 }
 return lfsr;
 }
//===========================================================================================================
//===========================================================================================================
float ComputePNDiffsTwoSeedsSC(int max_PNDiffs, float PNR[max_PNDiffs], float PNF[max_PNDiffs], float PND[max_PNDiffs], int LFSR_seed_low, int LFSR_seed_high)
{
   int initial_load_seed = 1;
   int i;
   uint16_t  lfsr_val_low, lfsr_val_high;
   for(i = 0; i < 2048; i++)
      {
        lfsr_val_low = LFSR_11_A_bits_low(initial_load_seed, (uint16_t)LFSR_seed_low);
	lfsr_val_high = LFSR_11_A_bits_high(initial_load_seed, (uint16_t)LFSR_seed_high);
  	PND[i] = PNR[lfsr_val_high] - PNF[lfsr_val_low];
	initial_load_seed = 0;
      }

   //return PND[];
}
//===========================================================================================================
//===========================================================================================================
void ComputePNDc(int max_PNDiffs, float PND[max_PNDiffs], float PNDc[max_PNDiffs], float reference_mean, float reference_range)
{
	int i;
	float zval_i;

	for (i = 0; i < 2048; i++)
	{
		zval_i = (PND[i] - (-32.883270))/55.9737;
		PNDc[i] = (zval_i * reference_range) + reference_mean;
	}
}
//===========================================================================================================
//===========================================================================================================
int GenGetTimingVals(volatile unsigned int *CtrlRegA, volatile unsigned int *DataRegA, int max_outputs, 
   int max_vecs_outputs, int max_sams, int start_index, unsigned short timing_val_arr[max_vecs_outputs][max_sams], 
   unsigned short output_pos[max_vecs_outputs][max_sams], int sam_num, int ctrl_mask)
   {
   int output_num, timing_val;
   int num_non_zero_timing_vals; 

//printf("GenGetTimingVals(): BEGIN\n"); fflush(stdout);

// Sanity check. PUF engine MUST be 'ready'
   if ( (*DataRegA & (1 << IN_SM_READY)) == 0 )
      { printf("ERROR: GenGetTimingVals(): PUF Engine is NOT ready!\n"); fflush(stdout); exit(EXIT_FAILURE); }

//printf("GenGetTimingVals(): PUF/hash engine is ready -- starting PUF engine!\n"); fflush(stdout);

// DEBUG: Set number of samples to 2.
//   ctrl_mask = ctrl_mask | (0 << OUT_CP_NUM_SAM1) | (0 << OUT_CP_NUM_SAM0);

// Start the PUF engine to generate the timing values. Vector pair has already been loaded.
   *CtrlRegA = ctrl_mask | (1 << OUT_CP_PUF_START);
   *CtrlRegA = ctrl_mask; 

//printf("GenGetTimingVals(): Waiting PUF engine to finish!\n"); fflush(stdout);

// Wait for the PUF engine to finish generating the timing data.
   while ( (*DataRegA & (1 << IN_SM_READY)) == 0 );

//printf("GenGetTimingVals(): PUF engine DONE!\n"); fflush(stdout);

// Reset the PN value pointers in the VHDL code for transferring PNs from VHDL to C code.
   *CtrlRegA = ctrl_mask | (1 << OUT_CP_DTI_RESTART); 
   *CtrlRegA = ctrl_mask; 

//printf("GenGetTimingVals(): Checking 'data_ready' of DataTransferOut!\n"); fflush(stdout);

// Wait for 'data_ready' to become 1 after the pointer reset (should already be 1).
   while ( (*DataRegA & (1 << IN_SM_DTI_DATA_READY)) == 0 );

//printf("GenGetTimingVals(): 'data_ready' is set!\n"); fflush(stdout);

// ==================
// Get timing values
   num_non_zero_timing_vals = 0;
   for ( output_num = 0; output_num < max_outputs; output_num++ )
      {

// Read a timing value -- current version uses only 10 bits (for timing values between 0 and 1023), even though phase
// shift can increase to 1120. NEED TO KEEP AN EYE ON THIS!!!!!!!!!!!!!!!!!!!!!!!! Save 400 FFs in the implementation
// right now. Once we move to SRAM, increase this to 11 bits!
      timing_val = *DataRegA & 0x7FF;

// If timing value is 0, then the path has NO transition.
      if ( timing_val > 0 )
         {

// Don't really need to save these since we are writing them to a file directly above.
         timing_val_arr[start_index + num_non_zero_timing_vals][sam_num] = (unsigned short)timing_val;
         output_pos[start_index + num_non_zero_timing_vals][sam_num] = (unsigned short)output_num;

//printf("GenGetTimingVals(): Timing val at index %d and sam %d is %d with output pos %d\n", 
//   start_index + num_non_zero_timing_vals, sam_num, 
//   timing_val_arr[start_index + num_non_zero_timing_vals][sam_num], 
//   output_pos[start_index + num_non_zero_timing_vals][sam_num]); fflush(stdout);

         num_non_zero_timing_vals++;

// Sanity check
         if ( num_non_zero_timing_vals + start_index > max_vecs_outputs )
            { 
            printf("ERROR: GenGetTimingVals(): Too many timing values %d -- max %d -- increase in program!\n", num_non_zero_timing_vals + start_index, max_vecs_outputs); 
            fflush(stdout); exit(EXIT_FAILURE); 
            }
         }

// Four phases here.
// 1) Got timing value above, indicate we are done reading.
      *CtrlRegA = ctrl_mask | (1 << OUT_CP_DTI_DONE_READING); 

// 2) Wait for 'data_ready' to become 0.
      while ( (*DataRegA & (1 << IN_SM_DTI_DATA_READY)) != 0 );

// 3) Reset done_reading to 0
      *CtrlRegA = ctrl_mask; 

// 4) Wait for 'data_ready' to become 1.
      while ( (*DataRegA & (1 << IN_SM_DTI_DATA_READY)) == 0 );
      }

   return num_non_zero_timing_vals;
   }


// ========================================================================================================
// ========================================================================================================
// Send the timing data to the verifier through the socket. THIS IS ONLY ALLOWED during enrollment.

void SendTimings(int str_length, int verifier_socket_desc, int vec_num, int num_timing_vals, int *num_timing_vals_ptr, 
   int max_outputs, int max_sams, unsigned short timing_val_arr[max_outputs][max_sams], 
   unsigned short output_pos[max_outputs][max_sams], int num_sams, int change_rise_fall)
   {
   char timing_line[str_length];
   char buffer[str_length];
   int sam_num, nz_num;
   int num_lines;

// Load up the timing samples to a string and send to the verifier.
   num_lines = 0;
   //printf("%d\n",vec_num);
   for ( nz_num = 0; nz_num < num_timing_vals; nz_num++ )
      {
      sprintf(timing_line, "V: %d\tO: %d\tC: %d\t", vec_num, output_pos[nz_num][0], *num_timing_vals_ptr);
      for ( sam_num = 0; sam_num < num_sams; sam_num++ )
         {
    	 if(*num_timing_vals_ptr == 1)
    	 {
    	 printf("%d\n",timing_val_arr[nz_num][sam_num]);
    	 }
         sprintf(buffer, " %d", timing_val_arr[nz_num][sam_num]);
         strcat(timing_line, buffer);
               
// Sanity check: It must always be true that the sampled output remains the same across samples.
         if ( output_pos[nz_num][0] != output_pos[nz_num][sam_num] )
            { printf("ERROR: SendTimings(): Output that produced timing value has CHANGED %d vs %d!\n", output_pos[nz_num][0], output_pos[nz_num][sam_num]); fflush(stdout); exit(EXIT_FAILURE); }
         }
      //exit(0);
         
// Send timing value line with all samples included. Be sure to add '+1' to ensure NULL character is transferred to receiver.
      if ( SockSendB((unsigned char *)timing_line, strlen(timing_line) + 1, verifier_socket_desc) < 0 )
         { printf("ERROR: SendTimings(): Send '%s' failed\n", timing_line); fflush(stdout); exit(EXIT_FAILURE); }

// DEBUG
//printf("SendTimings(): Current timing line[%d]:\n'%s'\n", *num_timing_vals_ptr, timing_line); fflush(stdout);
      num_lines++;
      (*num_timing_vals_ptr)++;
      }

// Send the empty string which will cause a blank line to be inserted into the saved file on the receiver, as an indicator of when we change 
// from rising to falling (or vise versa). Be sure to add '+1' to ensure NULL character is transferred to receiver.
   if ( change_rise_fall == 1 )
      if ( SockSendB((unsigned char *)"", strlen("") + 1, verifier_socket_desc) < 0 )
         { printf("ERROR: SendTimings(): Send '<cr>' failed\n"); fflush(stdout); exit(EXIT_FAILURE); }

// DEBUG
#ifdef DEBUG
printf("SendTimings(): Sent %d timing values for vector[%d]\n", nz_num, vec_num); fflush(stdout);
#endif
   }




// ========================================================================================================
// ========================================================================================================
// ========================================================================================================
// ========================================================================================================

// Looks like the full blown bitstreams are 4,045,564 in size
#define MAX_PR_BIN_SIZE  5000000

// Only need to be big enough to hold the max number of timing values per vector in this version.
unsigned short timing_val_arr[MAX_OUTPUTS][MAX_SAMS];
unsigned short output_pos[MAX_OUTPUTS][MAX_SAMS];

unsigned char *first_vecs_b[MAX_VECS];
unsigned char *second_vecs_b[MAX_VECS];
unsigned char *masks[MAX_VECS];

// Add as many as needed. These are the arrays which store the programming bitstreams.
char bstream_buf0[MAX_PR_BIN_SIZE];
int bstream_size0 = 0;
int bstream_loaded0 = 0;

int main(int argc, char *argv[])
   {
   volatile unsigned int *CtrlRegA;
   volatile unsigned int *DataRegA;

   char verifier_IP[MAX_STRING_LEN];
   int token_socket_desc = 0;
   int verifier_socket_desc = 0;
   int port_number;
   
   char chip_name[MAX_STRING_LEN];
   char outfile_name[MAX_STRING_LEN];

   int num_vecs, num_sams; 
   int vec_num, sam_num;
   int max_PNDiffs = 2048;
   float PNR[10003];
   float PNF[10007];
   float PND[2048];


//Timing variables.
#if DEBUG
   struct timeval t0, t1;
   long elapsed; 
#endif

   char bsfilename[MAX_STRING_LEN];
   int BSTREAM_FILE;
   int DEVCFG_FILE; 

   int num_timing_vals, num_rise_vecs;
   int first_num_non_zero_timing_vals, num_non_zero_timing_vals;
   int ctrl_mask;
   int has_masks; 

// ======================================================================================================================
// COMMAND LINE
   if ( argc != 3 )
      {
      printf("ERROR: token_enrollment.elf(): Chip name (C1)-- verifier_IP\n");
      return(1);
      }

   sscanf(argv[1], "%s", chip_name);
   strcpy(verifier_IP, argv[2]);

// ====================================================== PARAMETERS ====================================================
   num_sams = 16;
   port_number = 8888;
// ======================================================================================================================
// ======================================================================================================================
//Read the BitStream stored in SD card
/*
   strcpy(bsfilename, "/mnt/KG_SBOX_enroll.bit.bin");
   if ( strlen(bsfilename) == 0 )
      { printf("ERROR: SET VALID bitstream filename!\n"); fflush(stdout); exit(EXIT_FAILURE); }
         
   if( bstream_loaded0 == 0 )
      {   
      if ( (BSTREAM_FILE = open(bsfilename, O_RDONLY)) < 0 )
         { printf("ERROR: Failed to open bstream file '%s'!\n", bsfilename); fflush(stdout); exit(EXIT_FAILURE); }
      if ( (bstream_size0 = read(BSTREAM_FILE, bstream_buf0, MAX_PR_BIN_SIZE)) < 0 )
         { printf("ERROR: Failed to read bstream file '%s'!\n", bsfilename); fflush(stdout); exit(EXIT_FAILURE); }
      bstream_loaded0 = 1;
      close(BSTREAM_FILE);
      } 
// Program the PL side with the buffered bitstream
      if ( (DEVCFG_FILE = open("/dev/xdevcfg", O_RDWR)) < 0 )
         { printf("ERROR: Failed to open 'xdevcfg'!\n"); fflush(stdout); exit(EXIT_FAILURE); }
      write(DEVCFG_FILE, bstream_buf0, bstream_size0);
      close(DEVCFG_FILE);
      usleep (10000);
*/  
   freopen("output.log", "w", stderr);

// Open up the memory mapped device so we can access the GPIO registers.
   int fd = open("/dev/mem", O_RDWR|O_SYNC);

   if (fd < 0) 
      { printf("ERROR: /dev/mem could NOT be opened!\n"); exit(EXIT_FAILURE); }

// Add 2 for the DataReg (for an offset of 8 bytes for 32-bit integer variables)
   DataRegA = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_0_BASE_ADDR);
   CtrlRegA = DataRegA + 2;

// Open up a socket connection to the verifier.
   OpenSocketClient(MAX_STRING_LEN, verifier_IP, port_number, &verifier_socket_desc);

// Read all the vectors from the verifier into a set of string arrays. Verifier will send number of rising
// vectors (inspects vectors as it reads them) and indicate whether masks will also be sent.
   num_vecs = ReceiveVectors(MAX_STRING_LEN, verifier_socket_desc, MAX_VECS, first_vecs_b, second_vecs_b, VEC_LEN_BITS, 
      &num_rise_vecs, &has_masks, MAX_OUTPUTS, masks);


#ifdef DEBUG
printf("Number of vectors received %d\tNumber of rising vectors %d\tHas masks ? %d\n", num_vecs, num_rise_vecs, has_masks); fflush(stdout);
#endif

// Set the control mask to indicate enrollment (also, normally set PUF mode here too but there is only one mode here).
   ctrl_mask = (1 << OUT_CP_MODE1)|(1<<OUT_CP_MODE0); 
      
   strcpy(outfile_name, chip_name);
   strcat(outfile_name, "_PNs.txt");  

// Be sure to add '+1' to ensure NULL character is transferred to receiver.
   if ( SockSendB((unsigned char *)outfile_name, strlen(outfile_name) + 1, verifier_socket_desc) < 0 )
      { printf("ERROR: Send '%s' failed\n", outfile_name); fflush(stdout); exit(EXIT_FAILURE); }

// Run time information 
#ifdef DEBUG
   gettimeofday(&t0, 0);
#endif

// Do a soft RESET
   *CtrlRegA = (1 << OUT_CP_RESET);
   *CtrlRegA = 0;
   usleep (10000);

// ==================
// ==================
// Repeat for n vectors. NOTE: Enrollment mode of operation allows the PUF to be run on a per-vector basis, for as 
// many samples as you want (CollectPNs.vhd is NOT RUN -- ONLY LCDT). This iterative version allows very large sets 
// of timing values to be retrieved since the timing values for each vector are sent after they are generated. 

   num_timing_vals = 0;
   float mean_value = 0;
   for ( vec_num = 0; vec_num < num_vecs; vec_num++ )
      {
#ifdef DEBUG
      printf("Processing vector number %d\n", vec_num); fflush(stdout);
#endif

// Transfer a vector pair to the VHDL registers.
      LoadVecPairMask(MAX_STRING_LEN, CtrlRegA, DataRegA, MAX_VECS, vec_num, first_vecs_b, second_vecs_b, ctrl_mask, 
         VEC_LEN_BITS, VEC_CHUNK_SIZE, has_masks, MAX_OUTPUTS, masks);

// Repeat for n samples.
      for ( sam_num = 0; sam_num < num_sams; sam_num++ )
         {

// 'GenGetTimingVals' checks if PUF engine is ready, starts the engine, waits for strobing to complete and then 
// retrieves the timing data from the PUF.
         num_non_zero_timing_vals = GenGetTimingVals(CtrlRegA, DataRegA, MAX_OUTPUTS, MAX_OUTPUTS, MAX_SAMS, 0, 
            timing_val_arr, output_pos, sam_num, ctrl_mask);

// Sanity check. Same paths must be timed across ALL 16 samples.
         if ( sam_num == 0 )
            first_num_non_zero_timing_vals = num_non_zero_timing_vals;
         else if ( first_num_non_zero_timing_vals != num_non_zero_timing_vals )
            { printf("ERROR: Number of paths timed for first sample is NOT equal for other samples!\n"); fflush(stdout); exit(EXIT_FAILURE); }
         }

// Dump out the data to the verifier through the accepted socket. Last parameter adds a <cr> (blank line) to the 
// timing values sent back to the verifier after this last rising edge vector is applied.

      /*SendTimings(MAX_STRING_LEN, verifier_socket_desc, vec_num, num_non_zero_timing_vals, &num_timing_vals,
         MAX_OUTPUTS, MAX_SAMS, timing_val_arr, output_pos, num_sams, (vec_num == num_rise_vecs - 1));*/

      int nz_num;
      int num_lines = 0;
 	  int sum_16_delays = 0;
      float avg_16_delays = 0;


      for ( nz_num = 0; nz_num < num_non_zero_timing_vals; nz_num++ )
         {
    	 sum_16_delays = 0;
    	 avg_16_delays = 0;
         //sprintf(timing_line, "V: %d\tO: %d\tC: %d\t", vec_num, output_pos[nz_num][0], *num_timing_vals_ptr);
    //Finding the average of 16 PNs
         for ( sam_num = 0; sam_num < num_sams; sam_num++ )
            {
            sum_16_delays = sum_16_delays + timing_val_arr[nz_num][sam_num];
       	 /*if(*num_timing_vals_ptr == 1)
       	 {
       	 printf("%d\n",timing_val_arr[nz_num][sam_num]);
       	 }
         sprintf(buffer, " %d", timing_val_arr[nz_num][sam_num]);
         strcat(timing_line, buffer);
         */
   // Sanity check: It must always be true that the sampled output remains the same across samples.
            if ( output_pos[nz_num][0] != output_pos[nz_num][sam_num] )
               { printf("ERROR: SendTimings(): Output that produced timing value has CHANGED %d vs %d!\n", output_pos[nz_num][0], output_pos[nz_num][sam_num]); fflush(stdout); exit(EXIT_FAILURE); }
            }
         avg_16_delays = sum_16_delays/16.0000;
  //Pushing PNRs and PNFs to their respective arrays
         if (num_timing_vals < 10003)
         {
            PNR[num_timing_vals] = avg_16_delays;
         }
         if (num_timing_vals >= 10003)
         {
             PNF[num_timing_vals - 10003] = avg_16_delays;
         }


   // Send timing value line with all samples included. Be sure to add '+1' to ensure NULL character is transferred to receiver.
         /*if ( SockSendB((unsigned char *)timing_line, strlen(timing_line) + 1, verifier_socket_desc) < 0 )
            { printf("ERROR: SendTimings(): Send '%s' failed\n", timing_line); fflush(stdout); exit(EXIT_FAILURE); }*/

   // DEBUG
   //printf("SendTimings(): Current timing line[%d]:\n'%s'\n", *num_timing_vals_ptr, timing_line); fflush(stdout);
         num_lines++;
         (num_timing_vals)++;
         }
      }
   ComputePNDiffsTwoSeedsSC(max_PNDiffs, PNR, PNF, PND, 0, 0);
   int i;
   for(i = 0; i < 2048; i++)
{
	   mean_value = mean_value + PND[i];
}
   mean_value = mean_value/2048;
   float std_dev = 0;
   printf("%f\n",mean_value);
   for(i = 0; i < 2048; i++)
   {
	   std_dev = std_dev + ((PND[i] - mean_value)*(PND[i] - mean_value));
   }
   std_dev = std_dev/2048;
   std_dev = sqrt(std_dev);
   printf("%f\n",std_dev);

// Run time information
#ifdef DEBUG
   gettimeofday(&t1, 0); elapsed = (t1.tv_sec-t0.tv_sec)*1000000 + t1.tv_usec-t0.tv_usec; printf("\tDone: Elapsed %ld us\n\n", (long)elapsed);
#endif

// Send run time information ending with a "DONE" string. Be sure to add '+1' to ensure NULL character is transferred to receiver.
   if ( SockSendB((unsigned char *)"DONE", strlen("DONE") + 1, verifier_socket_desc) < 0 )
      { printf("ERROR: Send 'DONE' failed\n"); fflush(stdout); exit(EXIT_FAILURE); }
   close(verifier_socket_desc);
   close(token_socket_desc);
   fclose(stderr);
   return 0;
   } 
