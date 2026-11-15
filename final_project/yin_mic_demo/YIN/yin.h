#define YIN_TAU_MIN 50
#define YIN_THRESHOLD 0.10f // paper's value; try 0.15f if too many -1 results

typedef struct
{
  int16_t sample_size; // size of the audio buffer
  int16_t W;           // integration window size
  float *diff;         // d_t difference function
  float *cmndf_diff;   // d'_t difference function
  float abs_threshold; // absolute threshold (step 4 in paper)
  float periodicity;   //
  int16_t tau_max;     // max lag size
} Yin;

typedef struct
{
  float periodicity;
  float pitch;
} YinOut;

YinOut yin_algorithm(Yin *yin, int16_t *samples);

Yin yin_init(int16_t sample_size, int16_t W, float threshold, int16_t tau_max);