#ifndef 	__TOUCHSCREEN_FW__
#define 	__TOUCHSCREEN_FW__

#if defined(CONFIG_BOARD_AVON)
#define SYN_TPK_FW_NAME		"P864A10_TPK_PR1343768-s3202_zte_34313131.img"
#elif defined(CONFIG_BOARD_BARRON)
#define SYN_TPK_FW_NAME		"TPK_Z755_ZTE_PR1469071-s3408br_41313045.img"
#else

#define SYN_TPK_FW_NAME		""
#define SYN_TURLY_FW_NAME	"" 
#define SYN_SUCCESS_FW_NAME	""
#define SYN_OFILM_FW_NAME	""
#define SYN_LEAD_FW_NAME	""
#define SYN_WINTEK_FW_NAME	""
#define SYN_LAIBAO_FW_NAME	""
#define SYN_CMI_FW_NAME		""
#define SYN_ECW_FW_NAME		""
#define SYN_GOWORLD_FW_NAME	""
#define SYN_BAOMING_FW_NAME	""
#define SYN_JUNDA_FW_NAME	""

#define FTC_TPK_FW_NAME		""
#define FTC_TURLY_FW_NAME	"" 
#define FTC_SUCCESS_FW_NAME	""
#define FTC_OFILM_FW_NAME	""
#define FTC_LEAD_FW_NAME	""
#define FTC_WINTEK_FW_NAME	""
#define FTC_LAIBAO_FW_NAME	""
#define FTC_CMI_FW_NAME		""
#define FTC_ECW_FW_NAME		""
#define FTC_GOWORLD_FW_NAME	""
#define FTC_BAOMING_FW_NAME	""
#define FTC_JUNDA_FW_NAME	""
#define FTC_JIAGUAN_FW_NAME	""
#define FTC_MUDONG_FW_NAME	""

#endif

#ifndef SYN_TPK_FW_NAME
#define SYN_TPK_FW_NAME		""
#endif
#ifndef SYN_TURLY_FW_NAME
#define SYN_TURLY_FW_NAME	"" 
#endif
#ifndef SYN_SUCCESS_FW_NAME
#define SYN_SUCCESS_FW_NAME	"" 
#endif
#ifndef SYN_OFILM_FW_NAME
#define SYN_OFILM_FW_NAME	"" 
#endif
#ifndef SYN_LEAD_FW_NAME
#define SYN_LEAD_FW_NAME	"" 
#endif
#ifndef SYN_WINTEK_FW_NAME
#define SYN_WINTEK_FW_NAME	"" 
#endif
#ifndef SYN_LAIBAO_FW_NAME
#define SYN_LAIBAO_FW_NAME	"" 
#endif
#ifndef SYN_CMI_FW_NAME
#define SYN_CMI_FW_NAME		"" 
#endif
#ifndef SYN_ECW_FW_NAME
#define SYN_ECW_FW_NAME		"" 
#endif
#ifndef SYN_GOWORLD_FW_NAME
#define SYN_GOWORLD_FW_NAME	"" 
#endif
#ifndef SYN_BAOMING_FW_NAME
#define SYN_BAOMING_FW_NAME	"" 
#endif
#ifndef SYN_JUNDA_FW_NAME
#define SYN_JUNDA_FW_NAME	"" 
#endif
#ifndef FTC_TPK_FW_NAME
#define FTC_TPK_FW_NAME		""
#endif
#ifndef FTC_TURLY_FW_NAME
#define FTC_TURLY_FW_NAME	"" 
#endif
#ifndef FTC_SUCCESS_FW_NAME
#define FTC_SUCCESS_FW_NAME	"" 
#endif
#ifndef FTC_OFILM_FW_NAME
#define FTC_OFILM_FW_NAME	"" 
#endif
#ifndef FTC_LEAD_FW_NAME
#define FTC_LEAD_FW_NAME	"" 
#endif
#ifndef FTC_WINTEK_FW_NAME
#define FTC_WINTEK_FW_NAME	"" 
#endif
#ifndef FTC_LAIBAO_FW_NAME
#define FTC_LAIBAO_FW_NAME	"" 
#endif
#ifndef FTC_CMI_FW_NAME
#define FTC_CMI_FW_NAME		"" 
#endif
#ifndef FTC_ECW_FW_NAME
#define FTC_ECW_FW_NAME		"" 
#endif
#ifndef FTC_GOWORLD_FW_NAME
#define FTC_GOWORLD_FW_NAME	"" 
#endif
#ifndef FTC_BAOMING_FW_NAME
#define FTC_BAOMING_FW_NAME	"" 
#endif
#ifndef FTC_JUNDA_FW_NAME
#define FTC_JUNDA_FW_NAME	"" 
#endif
#ifndef FTC_JIAGUAN_FW_NAME
#define FTC_JIAGUAN_FW_NAME	"" 
#endif
#ifndef FTC_MUDONG_FW_NAME
#define FTC_MUDONG_FW_NAME	"" 
#endif
/*
固件名按照这个顺序排列
*/
enum TOUCH_MOUDLE
{
	TPK=0,
	TRULY,
	SUCCESS,
	OFILM,
	LEAD,
	WINTEK,
	LAIBAO,
	CMI,
	ECW,
	GOWORLD,
	BAOMING,
	JUNDA,
	JIAGUAN,
	MUDONG,
	UNKNOW=0xff
};
#define SYN_MOUDLE_NUM_MAX 12
#define FTC_MOUDLE_NUM_MAX 14

char *syn_fwfile_table[SYN_MOUDLE_NUM_MAX]=
{
SYN_TPK_FW_NAME,
SYN_TURLY_FW_NAME,
SYN_SUCCESS_FW_NAME,
SYN_OFILM_FW_NAME,
SYN_LEAD_FW_NAME,
SYN_WINTEK_FW_NAME,
SYN_LAIBAO_FW_NAME,
SYN_CMI_FW_NAME,
SYN_ECW_FW_NAME,
SYN_GOWORLD_FW_NAME,
SYN_BAOMING_FW_NAME,
SYN_JUNDA_FW_NAME
};

char *ftc_fwfile_table[FTC_MOUDLE_NUM_MAX]=
{
FTC_TPK_FW_NAME,
FTC_TURLY_FW_NAME,
FTC_SUCCESS_FW_NAME,
FTC_OFILM_FW_NAME,
FTC_LEAD_FW_NAME,
FTC_WINTEK_FW_NAME,
FTC_LAIBAO_FW_NAME,
FTC_CMI_FW_NAME,
FTC_ECW_FW_NAME,
FTC_GOWORLD_FW_NAME,
FTC_BAOMING_FW_NAME,
FTC_JUNDA_FW_NAME,
FTC_JIAGUAN_FW_NAME,
FTC_MUDONG_FW_NAME
};

int touch_moudle;
#endif

