#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "portaudio.h"
#include "fft/kiss_fftr.h"
#include <SDL.h>
#include <SDL_net.h>

#ifndef M_PI
#define M_PI (3.14159265)
#endif

#define SAMPLE_RATE (48000)
#define FRAMES_PER_BUFFER (1024)
#define FUNDAMENTAL_FREQ (16.0)
#define TABLE_SIZE (4800)

#define SCREEN_WIDTH  (1024)
#define SCREEN_HEIGHT (480)
#define N_POINTS_TIME (SCREEN_WIDTH) //TABLE_SIZE
#define X_SCALE_TIME (1)
#define Y_SCALE_TIME (SCREEN_HEIGHT)
#define N_POINTS_FREQ (SCREEN_WIDTH/2) //(FRAMES_PER_BUFFER/2)
#define X_SCALE_FREQ (2)
#define Y_SCALE_FREQ (8)

typedef enum {TIME_DOMAIN, FREQ_DOMAIN, TIME_DOMAIN_TEST} domain_t;
domain_t domain = 0;

SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;
SDL_Point points_time[N_POINTS_TIME];
SDL_Point points_freq[N_POINTS_FREQ];

kiss_fftr_cfg display_cfg;
kiss_fft_scalar display_in[FRAMES_PER_BUFFER];
kiss_fft_cpx display_out[FRAMES_PER_BUFFER];

#define N_WAVE_TYPES (5)
typedef enum {SINE, SQUARE, SAW, TRIANGLE, TEST} wave_t;
const char *wave_strings[] = {"sine", "square", "sawtooth", "triangle", "test"};

#define N_OCTAVES (9)
float table[N_WAVE_TYPES][N_OCTAVES][TABLE_SIZE];
//float ***table;
float volume = 0.1;

#define N_FREQS (108)
#define N_OSCILLATORS (N_FREQS)

#define N_ENV_STAGES (4)
typedef enum {ATTACK, DECAY, SUSTAIN, RELEASE} stage_t;

int MAX_ATTACK;
int MAX_DECAY;
int MAX_RELEASE;
typedef struct {
    float *attack;
    float *decay;
    float sustain;
    float *release;
} envelope;

typedef struct {
    double phase_index;
    double phase_step;
    float gain;
    wave_t type;
    int octave;
    stage_t envelope_stage;
    stage_t prev_stage;
    int envelope_index;
} oscillator;

struct shared_audio_data {
    oscillator *bank;
    envelope *env;
    double *table;
};

#define LINE_LEN (80)

typedef enum {INIT, KILL} pkt_cmd_t;
struct packet {
    pkt_cmd_t command;
    wave_t wave_type;
    Uint32 freq_index;
};
#define MAX_PEERS (4)
TCPsocket sock[MAX_PEERS+1];
int n_peers = 0;
int listening = 0;



void print_info(void)
{
    printf("\n=== CSYNTH ===\n");
    printf("\'tab\' swaps the display between time and frequency domains\n");
    printf("\'/\' cycles through waveforms\n");
    printf("\'<\' and \'>\' reduces and increases amplitude\n");
    printf("\'1\' to \'9\' selects octave\n");
    printf("\'q\' to \']\' plays notes\n");
    printf("\n");
}

void print_PA_err(PaError err)
{
    fprintf(stderr, "An error occured while using the portaudio stream.\n");
    fprintf(stderr, "PAError number: %d\n", err);
    fprintf(stderr, "PAError message: %s\n", Pa_GetErrorText(err));
}

char *get_ini_string(char *ini_buf, const char *target_string, FILE *ini_fp)
{
    char line[LINE_LEN];
    char *token;
    const char *delim = "= \r\n";
    while(fgets(line, LINE_LEN, ini_fp) != NULL) {
        token = strtok(line, delim);
        while(token != NULL) {
            if (strcmp(token, target_string) == 0) {
                token = strtok(NULL, delim);
                if (token == NULL) {
                    /* parameter was found but not set */
                    rewind(ini_fp);
                    return NULL;
                }
                else {
                    strncpy(ini_buf, token, LINE_LEN);
                    rewind(ini_fp);
                    return ini_buf;
                }
            }
            //printf("token: %s\n", token);
            token = strtok(NULL, delim);
        }
    }
    rewind(ini_fp);
    return NULL;
}



void refresh_signal(SDL_Renderer *renderer, SDL_Point *points, int n_points)
{
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLines(renderer, points, n_points); //SDL_RenderDrawPoints
    SDL_RenderPresent(renderer);
}

void update_points_time(SDL_Point *points, float *signal)
{
    int n = FRAMES_PER_BUFFER / 64;
    int i;
    for (i = 0; i < (N_POINTS_TIME - n); i++) {
        points[i].x = i * X_SCALE_TIME;
        points[i].y = points[i+n].y;
    }
    for (i = 1; i < n+1; i++) {
        points[N_POINTS_TIME - i].x = (N_POINTS_TIME - i) * X_SCALE_TIME;
        points[N_POINTS_TIME - i].y = (((signal[FRAMES_PER_BUFFER-(FRAMES_PER_BUFFER/n*i)]*-1.0) + 1) / 2.0) * Y_SCALE_TIME;        
    }
}

void update_points_freq(SDL_Point *points, kiss_fft_cpx *signal)
{
    float mag;
    int i;
    for (i = 0; i < FRAMES_PER_BUFFER/2; i++) {
        mag = sqrt((signal[i].r * signal[i].r) + (signal[i].i * signal[i].i));
        points[i].x = i * X_SCALE_FREQ ;
        points[i].y = mag * Y_SCALE_FREQ ;
    }
}



void calc_freq_table(double *freq_table)
{
    /* http://www.phy.mtu.edu/~suits/notefreqs.html */
    double a = pow(2.0, 1/12.0);
    freq_table[57] = 440;
    int i;
    for (i = 56; i >= 0; i--)
        freq_table[i] = freq_table[i+1] / a;
    for (i = 58; i < N_FREQS; i++)
        freq_table[i] = freq_table[i-1] * a;
    /*
    for (i = 0; i < 108; i++) {
        printf("%.2f\n", freq_table[i]);
    }
    printf("a:%.8f\n", a);
    */   
}

void calc_attack_envelope(envelope *env, int index)
{
    int i;
    for (i = 0; i < MAX_ATTACK; i++) {
        float pct = (float) i / MAX_ATTACK;
        env[index].attack[i] = pct;//*pct;
        //env[index].attack[i] = (i/(float)MAX_ATTACK);
    }
}

void calc_decay_envelope(envelope *env, int index, float start)
{
    int i;
    for (i = 0; i < MAX_DECAY; i++) {
        env[index].decay[i] = ((((MAX_DECAY-i-1)*(1-start)) / (float)MAX_DECAY) + start);
    }
}

void calc_release_envelope(envelope *env, int index, float start)
{
    float end = 0;
    int i;
    for (i = 0; i < MAX_RELEASE; i++) {
        float pct = (float) i / MAX_RELEASE;
        env[index].release[i] = start + (end-start)*sqrt(pct);
        //env[index].release[i] = ((((MAX_RELEASE-i-1)*start) / (float)MAX_RELEASE));
    }
} 

void calc_all_envelopes(envelope *env)
{
    int i;
    for (i = 0; i < N_OSCILLATORS; i++) {
        calc_attack_envelope(env, i);
        calc_decay_envelope(env, i, env[i].sustain);
        calc_release_envelope(env, i, env[i].sustain);
    }
}

int alloc_adsr(envelope *env, float attack_time_ms, float decay_time_ms, float sustain_pct, float release_time_ms)
{
    //printf("s:%.2f", sustain_pct);
    MAX_ATTACK = attack_time_ms / 1000 * SAMPLE_RATE;
    MAX_DECAY = decay_time_ms / 1000 * SAMPLE_RATE;
    MAX_RELEASE = release_time_ms / 1000 * SAMPLE_RATE;
    int i;
    for (i = 0; i < N_OSCILLATORS; i++) {
        env[i].attack = malloc(MAX_ATTACK * sizeof(float));
        env[i].decay = malloc(MAX_DECAY * sizeof(float));
        env[i].sustain = sustain_pct / 100;
        env[i].release = malloc(MAX_RELEASE * sizeof(float));
        if (!env[i].attack || !env[i].decay || !env[i].release)
            return 0;
        //printf("%.2f\n", env[i].sustain);
    }
    return 1;
}

void calc_distribution(double *profile, int bandwidth)
{
    int i;
    if (bandwidth != 1) {
        for (i = 0; i < bandwidth; i++) {
            //if bw = 7... x = 3, 2, 1, 0, 1, 2, 3
            int x = abs(i-(bandwidth/2));
            x /= (bandwidth/2);
            profile[i] = exp(-x*x);
        }
    } 
    else {
        profile[0] = 1.0;
    }
}

void calc_wavetable(double *freq_table, wave_t wave_type)
{
    time_t t;
    srand((unsigned) time(&t));
    int nyquist = TABLE_SIZE / 2 + 1;
    double base_freq = (double) SAMPLE_RATE / TABLE_SIZE;
    int fundamental_index = FUNDAMENTAL_FREQ / base_freq + 1;
    kiss_fft_cpx cpx_in[nyquist];
    kiss_fft_scalar scalar_out[TABLE_SIZE];
    kiss_fftr_cfg cfgi = kiss_fftr_alloc(TABLE_SIZE, 1, NULL, NULL);

    printf("Generating %s wavetables...\n", wave_strings[wave_type]);
    int i, j, k;
    //double spectrum[nyquist];
    //for (i = 0; i < nyquist; i++) spectrum[i] = 0;
    for (i = 0; i < N_OCTAVES; i++) {
        for (j = 0; j < nyquist; j++) {
            cpx_in[j].r = 0;
            cpx_in[j].i = 0;
            //printf("r:%.2f i:%.2f\n", cpx_out[j].r, cpx_out[j].i);
        }

        double highest_freq = freq_table[(i+1)*12-1];
        int bandlimit = (nyquist / (highest_freq / base_freq)) / 2 + 1;
        //printf("bandlimit: %d\n", bandlimit);
        double energy = 0;
        for (j = 1; j <= bandlimit; j++) {
            int bandwidth = 1;//sqrt(j*4);//j*4;
            if (bandwidth % 2 == 0) bandwidth++;
            double profile[bandwidth];
            calc_distribution(profile, bandwidth);
            int harm_index = fundamental_index * j;

            for (k = 0; k < bandwidth; k++) {
                double amplitude = 0;
                double phase = M_PI / 2.0;
                //double phase = ((double)rand()/RAND_MAX)*(M_PI*2);
                switch (wave_type) {
                    case SINE:
                        if (j == 1) amplitude = 1;
                        break;
                    case SQUARE:
                        if (j % 2 != 0) amplitude = 1.0 / j;
                        break;
                    case SAW:
                        amplitude = 1.0 / j;
                        break;
                    case TRIANGLE:
                        if (j % 2 != 0) amplitude = 1.0 / (j*j);
                        phase = M_PI;
                        break;
                    case TEST:
                        amplitude = 1.0 / sqrt(j);
                        break;
                }
                amplitude *= profile[k];
                //printf("amp: %.2f\n", amplitude);

                energy += amplitude;

                int spread_index = harm_index - bandwidth/2 + k;
                //if (k == bandwidth-1) printf("harm_i: %d, spread_i: %d\n", harm_index, spread_index);
                if (spread_index < nyquist) {
                    cpx_in[spread_index].r += amplitude * cos(phase);
                    cpx_in[spread_index].i += amplitude * sin(phase);
                    //spectrum[spread_index] = amplitude;
                }
                else {
                    fprintf(stderr, "out of bounds index: %d\n", spread_index);
                }
            }
            //printf("harmonic: %d\n", j);
        }

        //printf("energy: %.2f\n", sqrt(energy));
        kiss_fftri(cfgi, cpx_in, scalar_out);
        for (j = 0; j < TABLE_SIZE; j++) {
            table[wave_type][i][j] = scalar_out[j] / sqrt(energy) / 2; /// TABLE_SIZE;
            //printf("%.2f ", table[wave_type][i][j]);
        }
    }

    /*
    if (wave_type == SAW) {
        for (i = 0; i < nyquist; i++) fprintf(fp, "%.8f\n", spectrum[i]);
    }
    */

    free(cfgi);
}

void calc_all_wavetables(double *freq_table)
{
    int i;
    for (i = 0; i < N_WAVE_TYPES; i++) {
        calc_wavetable(freq_table, i);
    } 
}



void init_oscillator(oscillator *bank, int freq_index, float gain, wave_t type, double *freq_table, envelope *env)
{
    //bank[freq_index].phase_index = 0;
    bank[freq_index].phase_step = freq_table[freq_index] / FUNDAMENTAL_FREQ;
    bank[freq_index].gain = gain;
    bank[freq_index].octave = freq_index / 12;
    bank[freq_index].type = type;
    bank[freq_index].envelope_stage = ATTACK;
    int cur_index = bank[freq_index].envelope_index;
    if (cur_index > 0) {
        float cur_amp = env[freq_index].release[cur_index];
        bank[freq_index].envelope_index = (MAX_ATTACK * cur_amp); // depends on linear attack envelope
        //printf("%.2f, %.2f\n", cur_amp, env[freq_index].attack[bank[freq_index].envelope_index]);
    }
    else {
        bank[freq_index].envelope_index = 0;
    }
}

void kill_oscillator(oscillator *bank, int freq_index)
{
    bank[freq_index].envelope_stage = RELEASE;
    //bank[freq_index].envelope_index = 0;
}



float interpolate(oscillator osci)
{
    double p = osci.phase_index;
    int x1 = (int) p, x2;
    if (x1 == TABLE_SIZE-1)
        x2 = 0;
    else
        x2 = x1 + 1;
    //float y1 = table[data[j].type][x1], y2 = table[data[j].type][x2];
    float y1 = table[osci.type][osci.octave][x1], y2 = table[osci.type][osci.octave][x2];

    return (y1 + (y2 - y1) * (p - x1));
}

float proc_envelope(oscillator *bank, envelope *env, int i)
{
    float env_val;
    switch (bank[i].envelope_stage) {
        case ATTACK:
            //printf("ATTACK\n");
            env_val = env[i].attack[bank[i].envelope_index];
            if (bank[i].envelope_index == MAX_ATTACK - 1) {
                bank[i].envelope_stage = DECAY;
                bank[i].envelope_index = 0;
            }
            else {
                bank[i].envelope_index++;
            }
            bank[i].prev_stage = ATTACK;
            break;
        case DECAY:
            //printf("DECAY\n");
            env_val = env[i].decay[bank[i].envelope_index];
            if (bank[i].envelope_index == MAX_DECAY - 1) {
                bank[i].envelope_stage = SUSTAIN;
                bank[i].envelope_index = 0;
            }
            else {
                bank[i].envelope_index++;
            }
            bank[i].prev_stage = DECAY;
            break;
        case SUSTAIN:
            //printf("SUSTAIN\n");
            env_val = env[i].sustain;
            bank[i].prev_stage = SUSTAIN;
            break;
        case RELEASE:
            //printf("RELEASE\n");
            if (bank[i].prev_stage == ATTACK) {
                calc_release_envelope(env, i, env[i].attack[bank[i].envelope_index]);
                bank[i].envelope_index = 0;
                bank[i].prev_stage = RELEASE;
            }
            else if (bank[i].prev_stage == DECAY) {
                calc_release_envelope(env, i, env[i].decay[bank[i].envelope_index]);
                bank[i].envelope_index = 0;
                bank[i].prev_stage = RELEASE;
            }
            else if (bank[i].prev_stage == SUSTAIN) {
                calc_release_envelope(env, i, env[i].sustain);
                bank[i].envelope_index = 0;
                bank[i].prev_stage = RELEASE;
            }

            env_val = env[i].release[bank[i].envelope_index];

            if (bank[i].envelope_index == MAX_RELEASE - 1) {
                bank[i].gain = 0;
            }
            else {
                bank[i].envelope_index++;
            }
            break;
    }
    return env_val;
}

/* This routine will be called by the PortAudio engine when audio is needed.
 * It may called at interrupt level on some machines so don't do anything
 * that could mess up the system like calling malloc() or free().
 */
static int patestCallback(const void *inputBuffer, void *outputBuffer,
                          unsigned long framesPerBuffer,
                          const PaStreamCallbackTimeInfo* timeInfo,
                          PaStreamCallbackFlags statusFlags,
                          void *userData)
{
    struct shared_audio_data data = *((struct shared_audio_data*) userData);
    oscillator *bank = data.bank;
    envelope *env = data.env;
    float *out = (float*)outputBuffer;

    /* prevent unused variable warnings */
    (void) timeInfo;
    (void) statusFlags;
    (void) inputBuffer;

    int i, j;
    for (i = 0; i < framesPerBuffer; i++) {
        float samp = 0;
        for (j = 0; j < N_OSCILLATORS; j++) {
            if (bank[j].gain > 0) {
                float env_val = proc_envelope(bank, env, j);
                //printf("%.2f \n", env_val);
                samp += env_val * interpolate(bank[j]) * bank[j].gain * volume;
                bank[j].phase_index += bank[j].phase_step;
                if (bank[j].phase_index >= TABLE_SIZE) {
                    bank[j].phase_index -= TABLE_SIZE;
                }
            }
        }
        *out++ = samp;

        display_in[i] = samp;
    }

    if (domain == FREQ_DOMAIN) {
        kiss_fftr(display_cfg, display_in, display_out);
        update_points_freq(points_freq, display_out);
        refresh_signal(renderer, points_freq, N_POINTS_FREQ);
    }
    else if (domain == TIME_DOMAIN) {
        update_points_time(points_time, out-framesPerBuffer);
        refresh_signal(renderer, points_time, N_POINTS_TIME);
    }
    
    return paContinue;
}

PaError stop_PA(PaStream *stream)
{
    PaError err;
    err = Pa_StopStream(stream);
    if (err != paNoError) return err;

    err = Pa_CloseStream(stream);
    if (err != paNoError) return err;

    Pa_Terminate();

    return paNoError;
}

PaError setup_PA(PaStream **stream, struct shared_audio_data *data)
{
    PaError err;
    err = Pa_Initialize();
    if (err != paNoError) return err;

    PaStreamParameters outputParameters;
    outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
    if (outputParameters.device == paNoDevice) {
      fprintf(stderr, "Error: No default output device.\n");
      return paNoDevice;
    }
    outputParameters.channelCount = 1; /* mono output */
    outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
    outputParameters.suggestedLatency = Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    err = Pa_OpenStream(stream,
                        NULL, /* no input */
                        &outputParameters,
                        SAMPLE_RATE,
                        FRAMES_PER_BUFFER,
                        paClipOff, /* we won't output out of range samples so don't bother clipping them */
                        patestCallback,
                        data);
    if (err != paNoError) return err;

    err = Pa_StartStream(*stream);
    if (err != paNoError) return err;

    return paNoError;
}



void print_peer_addr(TCPsocket peer_sock)
{
    if (peer_sock) {
        IPaddress *remote_ip = SDLNet_TCP_GetPeerAddress(peer_sock);
        if (!remote_ip) {
            fprintf(stderr, "SDLNet_TCP_GetPeerAddress Error: %s\n", SDLNet_GetError());
            return;
        }
        Uint32 addr = SDL_SwapBE32(remote_ip->host);
        Uint16 port = SDL_SwapBE16(remote_ip->port);
        printf("%u.%u.%u.%u:%hu", addr>>24, (addr>>16)&0xFF, (addr>>8)&0xFF, addr&0xFF, port);
    }
}

int setup_SDLNet(const char *host, Uint16 port)
{
    listening = (host == NULL) ? 1 : 0;
    if (SDLNet_Init() != 0) {
        fprintf(stderr, "SDLNet_Init Error: %s\n", SDLNet_GetError());
        return 0;
    }
    IPaddress ip;
    if (SDLNet_ResolveHost(&ip, host, port) != 0) {
        fprintf(stderr, "SDLNet_ResolveHost Error: %s\n", SDLNet_GetError());
        return 0;
    }
    sock[0] = SDLNet_TCP_Open(&ip);
    if (!sock[0]) {
        fprintf(stderr, "SDLNet_TCP_Open Error: %s\n", SDLNet_GetError());
        return 0;
    }
    else if (host != NULL) {
        printf("Connected to host at "); print_peer_addr(sock[0]); printf("\n");
        n_peers++;
    }
    else {
        printf("Waiting for connections on port %hu\n", SDL_SwapBE16(ip.port));
    }
    return 1;
}

void send_pkt(pkt_cmd_t cmd, int freq_index, wave_t wave_type)
{
    struct packet pkt;
    pkt.command = cmd;
    pkt.wave_type = wave_type;
    pkt.freq_index = freq_index;
    int i, start, len;
    if (listening) {
        start = 1;
        len = MAX_PEERS+1;
    }
    else {
        start = 0;
        len = 1;
    }
    for (i = start; i < len; i++) {
        if (sock[i] && SDLNet_TCP_Send(sock[i], &pkt, sizeof(pkt)) < sizeof(pkt)) {
            fprintf(stderr, "SDLNet_TCP_Send Error: %s\n", SDLNet_GetError());
        }
    }
}

void recv_pkt(oscillator *bank, double *freq_table, envelope *env, struct packet *pkt, SDLNet_SocketSet set, int index)
{
    if (sock[index]) {
        int result = SDLNet_TCP_Recv(sock[index], pkt, sizeof(*pkt));
        if (result <= 0) {
            if (listening) {
                print_peer_addr(sock[index]); printf(" has disconnected.\n");
            }
            else {
                printf("Disconnected from host.\n");
            }
            SDLNet_TCP_DelSocket(set, sock[index]);
            SDLNet_TCP_Close(sock[index]);
            sock[index] = NULL;
            n_peers--;
            //perror("SDLNet_TCP_Recv()");
            //SDL_Delay(500);
        }
        else {
            if (pkt->command == INIT) {
                init_oscillator(bank, pkt->freq_index, 1.0, pkt->wave_type, freq_table, env);
                printf("Received - %.2fhz %s wave\n", freq_table[pkt->freq_index], wave_strings[pkt->wave_type]);
            }
            else if (pkt->command == KILL) {
                kill_oscillator(bank, pkt->freq_index);
            }
            //printf("received command: %d, frequency: %.2f\n", 
                   //pkt.command, freq_table[pkt.freq_index]);
        }
    }
}

void forward_pkt(struct packet pkt, TCPsocket tcp_sock)
{
    int i;
    for (i = 1; i < MAX_PEERS+1; i++) {
        if (sock[i] && sock[i] != tcp_sock && SDLNet_TCP_Send(sock[i], &pkt, sizeof(pkt)) < sizeof(pkt)) {
            fprintf(stderr, "SDLNet_TCP_Send Error: %s\n", SDLNet_GetError());
        }
    }
}

static void net_thread(void *user_data)
{
    struct shared_audio_data data = *((struct shared_audio_data*) user_data);
    oscillator *bank = data.bank;
    envelope *env = data.env;
    double *freq_table = data.table;

    int done = 0;
    SDLNet_SocketSet set = SDLNet_AllocSocketSet(MAX_PEERS+1);
    if (!set) {
        fprintf(stderr, "SDLNet_AllocSocketSet Error: %s\n", SDLNet_GetError());
        done = 1;
    }
    if (SDLNet_TCP_AddSocket(set, sock[0]) == -1) {
        fprintf(stderr, "SDLNet_TCP_AddSocket Error: %s\n", SDLNet_GetError());
        done = 1;
    }
    while (!done) {
        Uint32 timeout = -1;
        int n_ready = SDLNet_CheckSockets(set, timeout);
        if (n_ready == -1) {
            break;
            //SDL_Delay(500);
            //continue;
        }

        if (n_ready && listening && SDLNet_SocketReady(sock[0])) {
            int i;
            for (i = 1; i < MAX_PEERS+1; i++) {
                if (sock[i] == NULL) {
                    //printf("i: %d\n", i);
                    break;
                }
            }
            sock[i] = SDLNet_TCP_Accept(sock[0]);
            if (!sock[i]) {
                fprintf(stderr, "SDLNet_TCP_Accept Error: %s\n", SDLNet_GetError());
            }
            if (sock[i] && SDLNet_TCP_AddSocket(set, sock[i]) == -1) {
                fprintf(stderr, "SDLNet_TCP_AddSocket Error: %s\n", SDLNet_GetError());
            }
            else if (sock[i]) {
                printf("Connected to "); print_peer_addr(sock[i]); printf("\n");
                n_peers++;
            }
        }

        if (n_ready && listening) {
            int i;
            for (i = 1; i < MAX_PEERS+1; i++) {
                if (SDLNet_SocketReady(sock[i])) {
                    struct packet pkt;
                    recv_pkt(bank, freq_table, env, &pkt, set, i);
                    forward_pkt(pkt, sock[i]);
                }
            }
        }
        else if (n_ready && !listening && SDLNet_SocketReady(sock[0])) {
            struct packet pkt;
            recv_pkt(bank, freq_table, env, &pkt, set, 0);
        }

    }
}



int main(int argc, char *argv[]) 
{
    const char *fn = "settings.ini";
    FILE *fp = fopen(fn, "r");
    if (fp == NULL) {
        fprintf(stderr, "Error opening %s\n", fn);
        perror("fopen()");
    }
    char ini_buf[LINE_LEN];
    //struct shared_net_data sdlnet_data
    //printf("Initilizing...\n");
    /*
    table = alloc_wavetables();
    if (table == NULL) {
        fprintf(stderr, "Problem allocating wavetable memory\n");
        return -1;
    }
    */
    oscillator bank[N_OSCILLATORS];
    memset(bank, 0, sizeof(oscillator)*N_OSCILLATORS);
    envelope env[N_OSCILLATORS];
    if (!alloc_adsr(env, atof(get_ini_string(ini_buf, "attack_time_ms", fp)), 
                         atof(get_ini_string(ini_buf, "decay_time_ms", fp)), 
                         atof(get_ini_string(ini_buf, "sustain_percent", fp)), 
                         atof(get_ini_string(ini_buf, "release_time_ms", fp)))) {
        fprintf(stderr, "Problem allocating ADSR envelope memory\n");
        return -1;
    }
    double freq_table[N_FREQS];

    struct shared_audio_data pa_data;
    pa_data.bank = bank;
    pa_data.env = env;
    pa_data.table = freq_table;

    calc_freq_table(freq_table);
    calc_all_wavetables(freq_table);
    calc_all_envelopes(env);
    //print_envelopes();
    display_cfg = kiss_fftr_alloc(FRAMES_PER_BUFFER, 0, NULL, NULL);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL could not initialize, SDL_Error: %s\n", SDL_GetError());
        return -1;
    }
    if (SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, SDL_RENDERER_PRESENTVSYNC, &window, &renderer) != 0) {
        fprintf(stderr, "Window and renderer could not be created, SDL_Error: %s\n", SDL_GetError());
        SDL_Quit();
        return -1;
    }
    
    //if (setup_SDLNet((argc > 1) ? argv[1] : NULL, (argc > 2) ? atoi(argv[2]) : 4231)) {
    if (setup_SDLNet(get_ini_string(ini_buf, "host_ip", fp), atoi(get_ini_string(ini_buf, "port", fp)))) {
        SDL_Thread *net_thread_p = SDL_CreateThread((SDL_ThreadFunction)net_thread, "Network Thread", &pa_data);
        if (!net_thread_p) {
            fprintf(stderr, "SDL could not create a new thread, SDL_Error: %s\n", SDL_GetError());
        }
    }

    PaStream *stream;
    PaError err = setup_PA(&stream, &pa_data);
    if (err != paNoError) {
        SDLNet_Quit();
        SDL_DestroyWindow(window);
        SDL_Quit();
        Pa_Terminate();
        print_PA_err(err);
        return -1;
    }

    print_info();


                   /* '[', '\', ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 
                    * 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 
                    * 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z' */
    int char_map[] = { 11,   0,  12,   0,   0,   0,   0,   0,   0,   0,   3,   
                        0,   0,   0,   8,   0,   0,   0,   0,   0,   9,  10,
                        1,   4,   0,   5,   7,   0,   2,   0,   6,   0};
    int octave_map[12];
    wave_t wave_type = 0;
    int octave = 1;
    int running = 1;
    SDL_Event e;
    while (running && SDL_WaitEvent(&e)) {
        const Uint8 *state = SDL_GetKeyboardState(NULL);
        int k = e.key.keysym.sym;
        switch (e.type) {
            case SDL_KEYDOWN:
                if (state[SDL_SCANCODE_COMMA] && volume > 0.01) {
                    volume -= 0.01;
                    printf("volume: %.2f\n", volume);
                }
                else if (state[SDL_SCANCODE_PERIOD] && volume < 0.99) {
                    volume += 0.01;
                    printf("volume: %.2f\n", volume);
                }

                if (!e.key.repeat) {
                    /* https://wiki.libsdl.org/SDLKeycodeLookup */
                    if (k > 48 && k < 58) {
                        octave = k - 48;
                        printf("octave: %d\n", octave);
                        //if (domain == TIME_DOMAIN_TEST) test_points_time(wave_type, octave-1);
                    }
                    else if (k == SDLK_SLASH) {
                        if (wave_type == N_WAVE_TYPES-1)
                            wave_type = 0;
                        else
                            wave_type++;
                        printf("wave: %s\n", wave_strings[wave_type]);
                        //if (domain == TIME_DOMAIN_TEST) test_points_time(wave_type, octave-1);
                    }
                    else if (k == SDLK_TAB) {
                        if (domain == TIME_DOMAIN) {
                            printf("domain: frequency\n");
                            domain = FREQ_DOMAIN;
                        }
                        else if (domain == FREQ_DOMAIN) {
                            printf("domain: time\n");
                            domain = TIME_DOMAIN;
                            //test_points_time(wave_type, octave-1);
                        }
                        else if (domain == TIME_DOMAIN_TEST) {
                            domain = TIME_DOMAIN;
                        }
                    }
                    //else if (k == SDLK_SPACE) {
                    //}
                    else if (k > 90 && k < 123) {
                        if (char_map[k-91] == 0)
                            break;
                        octave_map[char_map[k-91]-1] = octave;
                        int freq_index = (char_map[k-91]-1) + ((octave-1)*12);
                        printf("frequency: %.2f\n", freq_table[freq_index]);
                        init_oscillator(bank, freq_index, 1.0, wave_type, freq_table, env);
                        if (n_peers > 0)
                            send_pkt(INIT, freq_index, wave_type);
                    }
                }
                break;
            case SDL_KEYUP:
                if (k > 90 && k < 123) {
                    if (char_map[k-91] == 0)
                        break;
                    int freq_index = (char_map[k-91]-1) + ((octave_map[char_map[k-91]-1]-1)*12);
                    kill_oscillator(bank, freq_index);
                    if (n_peers > 0)
                        send_pkt(KILL, freq_index, 0);
                }
                break;
            case SDL_QUIT:
                running = 0;
        }
    }

    fclose(fp);

    kiss_fft_cleanup();

    SDLNet_Quit();
    SDL_DestroyWindow(window);
    SDL_Quit();

    err = stop_PA(stream);
    if (err != paNoError) {
        Pa_Terminate();
        print_PA_err(err);
        return -1;
    }

    printf("\nDone.\n");
    return 0;
}


#if 0
void test_points_time(wave_t wave_type, int octave)
{
    int i;
    for (i = 0; i < N_POINTS_TIME; i++) {
        points_time[i].x = i * X_SCALE_TIME;
        points_time[i].y = (table[wave_type][octave][(i+1)*(TABLE_SIZE/N_POINTS_TIME)-1]*-1.0 + 1) / 2.0 * (Y_SCALE_TIME);
    }
    refresh_signal(renderer, points_time, N_POINTS_TIME);
    /*
    int n = 10;
    printf("first %d samples:\n", n);
    for (i = 0; i < n; i++) printf("%.2f\n", table[wave_type][octave][i]);
    printf("last %d samples:\n", n);
    for (i = 0; i < n; i++) printf("%.2f\n", table[wave_type][octave][TABLE_SIZE-n+i]);
    */
}

float*** alloc_wavetables(void)
{
    float ***wt;
    wt = malloc(N_WAVE_TYPES * sizeof(float**));
    if (wt == NULL) return NULL;
    int i, j;
    for (i = 0; i < N_WAVE_TYPES; i++) {
        wt[i] = malloc(N_OCTAVES * sizeof(float*));
        if (wt[i] == NULL) return NULL;
        for (j = 0; j < N_OCTAVES; j++) {
            wt[i][j] = malloc(TABLE_SIZE * sizeof(float));
            if (wt[i][j] == NULL) return NULL;
        }
    }
    return wt;
}

void calc_wavetables(void)
{
    int i, j;
    for (i = 0; i < TABLE_SIZE; i++) {
        for (j = 0; j < N_OCTAVES; j++) {
            /* create sine wave */
            table[SINE][j][i] = sin((i/(double)TABLE_SIZE) * M_PI * 2.0);
            /* create square wave */
            if (i < TABLE_SIZE / 2)
                table[SQUARE][j][i] = 1.0;
            else 
                table[SQUARE][j][i] = -1.0;
            /* create sawtooth wave */
            table[SAW][j][i] = ((i/(double)TABLE_SIZE)*2) - 1;
        }
    }
}


void print_envelopes(void)
{
    int i;
    printf("\nATTACK\n");
    for (i = 0; i < MAX_ATTACK; i++) {
        printf("%.2f ", env.attack[i]);
    }
    printf("\nDECAY\n");
    for (i = 0; i < MAX_DECAY; i++) {
        printf("%.2f ", env.decay[i]);
    }
    printf("\nRELEASE\n");
    for (i = 0; i < MAX_RELEASE; i++) {
        printf("%.2f ", env.release[i]);
    }
}
#endif