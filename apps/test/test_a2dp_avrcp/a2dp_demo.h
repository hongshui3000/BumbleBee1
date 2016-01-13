#ifndef __A2DP_DEMO_H
#define __A2DP_DEMO_H
#define A2DP_SIG_SEID_SRC    1
#define A2DP_SIG_SEID_SNK    12
#define A2DP_STREAM_SEID_SRC    13
#define A2DP_STREAM_SEID_SNK1    14
#define A2DP_STREAM_SEID_SNK2    15   //wade

/* AVDTP TSEP                                                               */
#define AVDTP_TSEP_SRC                      0
#define AVDTP_TSEP_SNK                      1
/*AVDTP SAMPLEFREQUENCE */
#define  A2DP_FREQU16000   0x80
#define  A2DP_FREQU32000   0x40
#define  A2DP_FREQU44100   0x20
#define  A2DP_FREQU48000   0x10
/*AVDTP CHANNELMODE */
#define  A2DP_MODEMOMO      0x8
#define  A2DP_MODEDUAL      0x4
#define  A2DP_MODESTEREO    0x2
#define  A2DP_MODEJOINT     0x1
/*AVDTP BLOCK LENGTH*/
#define  A2DP_BLOCKS4     0x80
#define  A2DP_BLOCKS8     0x40
#define  A2DP_BLOCKS12    0x20
#define  A2DP_BLOCKS16    0x10
/*AVDTP SUBBAND */
#define  A2DP_SUBBANDS4   0x8
#define  A2DP_SUBBANDS8   0x4
/*AVDTP ALLOCATE METHOD */
#define  A2DP_ALLOCLOUDNESS   0x1
#define  A2DP_ALLOCSNR        0x2
/*AVDTP BITPOOL */
#define A2DP_MIN_BITPOOL    2
#define A2DP_MAX_BITPOOL    100

#define SBC_FREQU16000   0x0
#define SBC_FREQU32000   0x1
#define SBC_FREQU44100   0x2
#define SBC_FREQU48000   0x3

#define SBC_BLOCKS4    0x0
#define SBC_BLOCKS8    0x1
#define SBC_BLOCKS12   0x2
#define SBC_BLOCKS16   0x3

#define SBC_MODE_MONO     0x0
#define SBC_MODE_DUAL     0x1
#define SBC_MODE_STEREO   0x2
#define SBC_MODE_JOINT    0x3

#define SBC_ALLOCLOUDNESS  0x0
#define SBC_ALLOCSNR       0x1

#define SBC_SUBBANDS4  0x0
#define SBC_SUBBANDS8  0x1

bool App_Init(void);
void App_sim_cmd(char rx);
#endif

